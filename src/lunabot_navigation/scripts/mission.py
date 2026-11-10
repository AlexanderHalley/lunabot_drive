#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the competition mission: dig, dump, repeat, go home.

    ros2 run lunabot_navigation mission.py --ros-args \
        --params-file $(ros2 pkg prefix lunabot_navigation)/share/lunabot_navigation/config/\
mission_ucf.yaml

Waits for the start survey (/navigation/survey_done), then for each cycle
drives to the dig pose, digs, drives to the dump pose, dumps; then drives
home. Every phase change is latched on /mission/state as one line of text,
for operators, logs and the demo recorder.

Digging and dumping are goals to the excavation server (lunabot_excavation:
/excavation/excavate and /excavation/deposit, lunabot_msgs actions): dig for
up to `dig_seconds` or until `target_fill` where the fill is measured, dump
for up to `dump_seconds`. Each ends with the excavator stowed, so the next
leg may drive. Until the drum and hopper have drivers the server's backend is
a model that drives nothing -- the stops last as long as before, through the
interface the driver will serve.

A failed leg or a faulted dig/dump is retried `retries` times (Nav2 replans
from wherever the rover ended up); one that still fails ends the mission as
FAILED rather than skipping ahead, because the next step starts from the
wrong place.

Operator control (docs/MONITORING.md "Mission control"): std_srvs/Trigger
services /mission/start, /mission/pause, /mission/resume, /mission/abort, and
the software e-stop, std_msgs/Bool on /e_stop (lunabot_bringup's
operator_e_stop.py) -- twist_mux's lock cuts every velocity source, and this
node pauses (see Control). /mission/mode is one word
for indicator lights: READY, RUNNING, PAUSED, E-STOP, ABORTED, FAILED, COMPLETE.
With autostart false the mission waits in READY for /mission/start, and
latches True on /mission/started when it gets it -- the start survey waits for
that when launched with wait_for_start:=true, so nothing moves before start.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STATE_TOPIC = '/mission/state'
MODE_TOPIC = '/mission/mode'
STARTED_TOPIC = '/mission/started'
SURVEY_TOPIC = '/navigation/survey_done'
E_STOP_TOPIC = '/e_stop'
EXCAVATE_ACTION = '/excavation/excavate'
DEPOSIT_ACTION = '/excavation/deposit'
# std_srvs/Trigger services, /mission/<name>, each calling Control.<name>().
COMMANDS = ('start', 'pause', 'resume', 'abort')


class Control:
    """The operator's commands, as state the sequencer polls. Pure: tested without ROS.

    Each command returns (accepted, message): the Trigger response. The e-stop
    pauses too, and releasing it does NOT resume -- motion restarts only on an
    explicit resume, once someone has looked. A pause cancels the Nav2 goal; on
    resume the interrupted leg is sent again from wherever the rover stopped,
    and a dig or dump carries on with the time it had left. Abort is final:
    restart the node to run again.
    """

    def __init__(self, autostart: bool):
        self.started = bool(autostart)
        self.paused = False
        self.e_stop = False
        self.final = ''  # ABORTED, FAILED or COMPLETE once over

    def _over(self):
        return False, f'mission {self.final.lower()}: restart mission.py to run again'

    def start(self):
        if self.final:
            return self._over()
        if self.started:
            return False, 'already started'
        if self.e_stop:
            return False, 'e-stop engaged: release it first'
        self.started, self.paused = True, False
        return True, 'started'

    def pause(self):
        if self.final:
            return self._over()
        if not self.started:
            return False, 'not started'
        if self.paused:
            return False, 'already paused'
        self.paused = True
        return True, 'paused'

    def resume(self):
        if self.final:
            return self._over()
        if self.e_stop:
            return False, 'e-stop engaged: release it first'
        if not self.paused:
            return False, 'not paused'
        self.paused = False
        return True, 'resumed'

    def abort(self):
        if self.final:
            return self._over()
        self.final = 'ABORTED'
        return True, 'aborted'

    def set_e_stop(self, engaged: bool) -> None:
        if engaged and not self.e_stop:
            self.paused = True
        self.e_stop = bool(engaged)

    @property
    def aborted(self) -> bool:
        return self.final == 'ABORTED'

    def mode(self) -> str:
        """Return /mission/mode: one word, for indicator lights."""
        if self.final:
            return self.final
        if self.e_stop:
            return 'E-STOP'
        if not self.started:
            return 'READY'
        return 'PAUSED' if self.paused else 'RUNNING'


class Aborted(Exception):
    """The operator aborted the mission."""


@dataclass(frozen=True)
class Step:
    """One thing the mission does: drive to a pose, or dig or dump for up to some seconds."""

    state: str
    pose: tuple[float, float, float] | None = None
    work: str = ''  # 'excavate' or 'deposit'
    seconds: float = 0.0


def work_outcome(name: str) -> str:
    """Map an Excavate/Deposit outcome (its constant's name) onto what the mission does.

    'done': carry on. 'held': stopped for the operator -- wait, then send the
    rest. 'e_stop': the excavator saw the e-stop first; the mission pauses as
    if it had. 'failed': counts against `retries`.
    """
    return {
        'FULL': 'done',
        'EMPTY': 'done',
        'TIME_LIMIT': 'done',
        'CANCELED': 'held',
        'E_STOP': 'e_stop',
    }.get(name, 'failed')


def plan(cycles, dig_pose, dump_poses, home_pose, dig_seconds, dump_seconds) -> list[Step]:
    """Return the mission as a flat list of steps. Pure: tested without ROS.

    dump_poses: where to park for each dump, used in turn and repeated, so
    successive loads go to different spots along the berm.
    """
    steps = []
    for i in range(1, int(cycles) + 1):
        tag = f' ({i}/{cycles})' if cycles > 1 else ''
        steps += [
            Step('DRIVING TO EXCAVATION ZONE' + tag, pose=tuple(dig_pose)),
            Step('EXCAVATING' + tag, work='excavate', seconds=dig_seconds),
            Step(
                'DRIVING TO CONSTRUCTION ZONE' + tag,
                pose=tuple(dump_poses[(i - 1) % len(dump_poses)]),
            ),
            Step('DEPOSITING' + tag, work='deposit', seconds=dump_seconds),
        ]
    steps.append(Step('RETURNING TO START', pose=tuple(home_pose)))
    return steps


def park_pose(target, reach):
    """Return where to park so the load lands on target = (x, y, heading).

    The excavator pours `reach` metres BEHIND base_link, so the rover parks
    that far ahead of the target point, facing `heading`.
    """
    x, y, yaw = target
    return (x + reach * math.cos(yaw), y + reach * math.sin(yaw), yaw)


def dump_targets(flat):
    """Split a flat [x, y, heading, x, y, heading, ...] list into triples."""
    if not flat or len(flat) % 3:
        raise ValueError('dump_targets needs x, y, heading per spot')
    return [tuple(flat[i : i + 3]) for i in range(0, len(flat), 3)]


def to_map(pose, fix):
    """Convert an arena-frame (x, y[, yaw]) into map, given the map origin's pose in arena.

    A pose without a yaw stays without one: its heading is free (arrive_heading()).
    """
    fx, fy, fyaw = fix
    dx, dy = pose[0] - fx, pose[1] - fy
    c, s = math.cos(fyaw), math.sin(fyaw)
    xy = (c * dx + s * dy, -s * dx + c * dy)
    if len(pose) == 2:
        return xy
    return xy + (math.remainder(pose[2] - fyaw, math.tau),)


def arrive_heading(goal, start) -> float:
    """Return the heading for a goal given as [x, y]: the way the rover drives there.

    For goals where the heading does not matter (the dig spot), so the rover
    is not asked to turn round on arrival -- arriving from the far side and
    made to face a fixed way, it planned small loops on the spot.
    """
    if len(goal) == 3:
        return goal[2]
    return math.atan2(goal[1] - start[1], goal[0] - start[0])


def main() -> None:
    import time

    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    from lunabot_msgs.action import Deposit, Excavate

    rclpy.init()
    node = Node('mission')
    p = {
        name: node.declare_parameter(name, default).value
        for name, default in (
            ('cycles', 1),
            ('leg_timeout', 240.0),
            ('retries', 1),
            ('dig_pose', [4.0, 0.8]),
            ('dump_targets', [5.95, 2.57, -math.pi / 2, 6.23, 2.57, -math.pi / 2]),
            ('dump_reach', 0.97),
            ('home_pose', [0.4, 0.0]),
            # Limits on the excavator's working time: drum digging, hopper
            # tipped (lowering, raising and tipping come on top).
            ('dig_seconds', 8.0),
            ('dump_seconds', 6.0),
            # Dig until this fill instead, where the excavator measures it.
            ('target_fill', 1.0),
            # Wall-clock limit on one dig or dump goal, as leg_timeout.
            ('work_timeout', 90.0),
            ('wait_for_survey', True),
            # The poses above are in this frame. 'arena' needs the start fix
            # (fiducial_localizer.py: arena -> map); 'map' sends them as they are.
            ('frame', 'arena'),
            ('fix_timeout', 15.0),
            # False: wait in READY for /mission/start (the competition run).
            ('autostart', True),
        )
    }
    tf = Buffer()
    TransformListener(tf, node)
    log = node.get_logger()
    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    state_pub = node.create_publisher(String, STATE_TOPIC, latched)
    mode_pub = node.create_publisher(String, MODE_TOPIC, latched)
    started_pub = node.create_publisher(Bool, STARTED_TOPIC, latched)
    control = Control(p['autostart'])
    shown = {'mode': None}

    def publish_mode() -> None:
        mode = control.mode()
        if mode != shown['mode']:
            shown['mode'] = mode
            mode_pub.publish(String(data=mode))

    def command(name):
        def serve(_request, response):
            response.success, response.message = getattr(control, name)()
            log.info(f'/mission/{name}: {response.message}')
            publish_mode()
            return response

        return serve

    for name in COMMANDS:
        node.create_service(Trigger, f'/mission/{name}', command(name))
    # Default QoS, as twist_mux's lock subscribes: the same /e_stop message
    # stops the wheels there and pauses the mission here.
    node.create_subscription(
        Bool, E_STOP_TOPIC, lambda m: (control.set_e_stop(m.data), publish_mode()), 10
    )

    def say(state: str) -> None:
        log.info(f'mission: {state}')
        state_pub.publish(String(data=state))

    def spin(seconds: float = 0.05) -> None:
        rclpy.spin_once(node, timeout_sec=seconds)
        publish_mode()

    def hold(state: str) -> None:
        """Wait out a pause or an e-stop, saying so; raise Aborted on abort."""
        held = None
        while rclpy.ok() and control.paused and not control.final:
            label = f'{"E-STOP" if control.e_stop else "PAUSED"}: {state}'
            if label != held:
                say(label)
                held = label
            spin()
        if control.aborted:
            raise Aborted(state)
        if held:
            say(state)

    def work(step, left: float):
        """Dig or dump for up to `left` s: ('done' | 'failed' | 'held', seconds still left)."""
        client, kind = excavator[step.work]
        goal = kind.Goal(max_seconds=float(max(left, 0.0)))
        if step.work == 'excavate':
            goal.target_fill = float(p['target_fill'])
        sent = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            log.error(f'{step.work} goal refused by the excavator')
            return 'failed', left
        result = handle.get_result_async()
        deadline = time.monotonic() + p['work_timeout']
        while not result.done():
            if control.paused or control.final:
                handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, result, timeout_sec=5.0)
                break
            if time.monotonic() > deadline:
                log.error(f'{step.work} timed out')
                handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, result, timeout_sec=5.0)
                return 'failed', left
            spin()
        if not result.done():
            return 'held', left
        r = result.result().result
        names = ('FULL', 'EMPTY', 'TIME_LIMIT', 'CANCELED', 'E_STOP', 'FAULT')
        name = next((n for n in names if getattr(type(r), n, None) == r.outcome), str(r.outcome))
        left -= r.seconds
        log.info(f'{step.work}: {name} after {r.seconds:.1f} s ({r.message})')
        outcome = work_outcome(name)
        if outcome == 'e_stop':
            control.set_e_stop(True)  # /e_stop's release clears it, as for the mission's own
            publish_mode()
            outcome = 'held'
        return outcome, left

    def drive(pose) -> str:
        """Drive to pose: 'done', 'failed', or 'held' (paused or aborted on the way)."""
        yaw = pose[2] if len(pose) == 3 else None
        if yaw is None:
            try:
                t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                here = (t.transform.translation.x, t.transform.translation.y)
            except Exception:  # noqa: BLE001
                here = (0.0, 0.0)
                log.warning('no map -> base_link: heading for a free-heading goal from (0, 0)')
            yaw = arrive_heading(pose, here)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        sent = nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            log.error('goal rejected')
            return 'failed'
        result = handle.get_result_async()
        # Wall clock, as before: a leg that hangs because /clock stopped
        # should still time out.
        deadline = time.monotonic() + p['leg_timeout']
        while not result.done():
            if control.paused or control.final:
                rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5.0)
                return 'held'
            if time.monotonic() > deadline:
                log.error('leg timed out')
                handle.cancel_goal_async()
                return 'failed'
            spin()
        return 'done' if result.result().status == 4 else 'failed'  # STATUS_SUCCEEDED

    def wait_for_fix(frame):
        from rclpy.time import Time

        end = node.get_clock().now().nanoseconds + int(p['fix_timeout'] * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            try:
                t = tf.lookup_transform(frame, 'map', Time())
            except Exception:  # noqa: BLE001
                spin(0.1)
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        return None

    def finish(final: str, state: str) -> None:
        control.final = final
        say(state)
        publish_mode()
        rclpy.spin(node)  # keep the latched state up and answer the services

    try:
        try:
            publish_mode()
            say('WAITING FOR NAVIGATION')
            nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
            while rclpy.ok() and not nav.wait_for_server(timeout_sec=0.5):
                spin()
            say('WAITING FOR EXCAVATOR')
            excavator = {
                'excavate': (ActionClient(node, Excavate, EXCAVATE_ACTION), Excavate),
                'deposit': (ActionClient(node, Deposit, DEPOSIT_ACTION), Deposit),
            }
            for client, _ in excavator.values():
                while rclpy.ok() and not client.wait_for_server(timeout_sec=0.5):
                    spin()
            if not control.started:
                say('READY: WAITING FOR START')
                while rclpy.ok() and not control.started and not control.final:
                    spin(0.1)
                if control.aborted:
                    raise Aborted('READY')
            started_pub.publish(Bool(data=True))
            if p['wait_for_survey']:
                survey = {'done': None}
                node.create_subscription(
                    Bool, SURVEY_TOPIC, lambda m: survey.__setitem__('done', m.data), latched
                )
                say('SURVEYING START AREA')
                while rclpy.ok() and survey['done'] is None and not control.final:
                    spin(0.2)
                if control.aborted:
                    raise Aborted('SURVEYING START AREA')
                if not survey['done']:
                    log.warning('the start survey failed; carrying on without it')

            fix = (0.0, 0.0, 0.0)
            if p['frame'] != 'map':
                say('LOCALISING ON APRILTAGS')
                fix = wait_for_fix(p['frame'])
                if fix is None:
                    fix = (0.0, 0.0, 0.0)
                    log.error(
                        f'no {p["frame"]} -> map after {p["fix_timeout"]:.0f} s: ASSUMING THE '
                        f'ROVER STARTED AT THE {p["frame"].upper()} ORIGIN. Right only for the '
                        'default sim spawn.'
                    )
                else:
                    log.info(
                        f'start fix: map origin at {p["frame"]} x {fix[0]:.2f} y {fix[1]:.2f} '
                        f'yaw {math.degrees(fix[2]):.0f} deg'
                    )

            steps = plan(
                p['cycles'],
                to_map(p['dig_pose'], fix),
                [
                    to_map(park_pose(t, p['dump_reach']), fix)
                    for t in dump_targets(p['dump_targets'])
                ],
                to_map(p['home_pose'], fix),
                p['dig_seconds'],
                p['dump_seconds'],
            )
            log.info(f'mission: {p["cycles"]} dig-and-dump cycle(s), poses in {p["frame"]}')
            start = node.get_clock().now()
            for step in steps:
                hold(step.state)
                say(step.state)
                failures = 0
                left = step.seconds  # a dig's or dump's time, kept across pauses
                while True:
                    hold(step.state)
                    if step.work:
                        outcome, left = work(step, left)
                    else:
                        outcome = drive(step.pose)
                    if outcome == 'done':
                        break
                    if outcome == 'held':  # an operator stop is not a failure
                        continue
                    failures += 1
                    if failures > p['retries']:
                        finish('FAILED', f'FAILED: {step.state}')
                        return
                    say(f'{step.state} - RETRYING')
            elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
            finish('COMPLETE', f'MISSION COMPLETE in {elapsed:.0f} s')
        except Aborted as stopped:
            finish('ABORTED', f'ABORTED: {stopped}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
