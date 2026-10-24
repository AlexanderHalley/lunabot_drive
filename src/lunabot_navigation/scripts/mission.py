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

Digging and dumping are PLACEHOLDERS -- a timed stop. There is no excavation
package yet; when there is, `_work()` calls its actions instead.

A failed leg is retried `retries` times (Nav2 replans from wherever the rover
ended up); a leg that still fails ends the mission as FAILED rather than
skipping ahead, because the next leg starts from the wrong place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STATE_TOPIC = '/mission/state'
SURVEY_TOPIC = '/navigation/survey_done'


@dataclass(frozen=True)
class Step:
    """One thing the mission does: drive to a pose, or work for some seconds."""

    state: str
    pose: tuple[float, float, float] | None = None
    seconds: float = 0.0


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
            Step('EXCAVATING' + tag, seconds=dig_seconds),
            Step(
                'DRIVING TO CONSTRUCTION ZONE' + tag,
                pose=tuple(dump_poses[(i - 1) % len(dump_poses)]),
            ),
            Step('DEPOSITING' + tag, seconds=dump_seconds),
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
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

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
            ('dig_seconds', 8.0),
            ('dump_seconds', 6.0),
            ('wait_for_survey', True),
            # The poses above are in this frame. 'arena' needs the start fix
            # (fiducial_localizer.py: arena -> map); 'map' sends them as they are.
            ('frame', 'arena'),
            ('fix_timeout', 15.0),
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

    def say(state: str) -> None:
        log.info(f'mission: {state}')
        state_pub.publish(String(data=state))

    def sleep(seconds: float) -> None:
        # Sim time: a stop of N seconds is N seconds on /clock.
        end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def drive(pose) -> bool:
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
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=p['leg_timeout'])
        if not result.done():
            log.error('leg timed out')
            handle.cancel_goal_async()
            return False
        return result.result().status == 4  # GoalStatus.STATUS_SUCCEEDED

    def wait_for_fix(frame):
        from rclpy.time import Time

        end = node.get_clock().now().nanoseconds + int(p['fix_timeout'] * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            try:
                t = tf.lookup_transform(frame, 'map', Time())
            except Exception:  # noqa: BLE001
                rclpy.spin_once(node, timeout_sec=0.1)
                continue
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return (t.transform.translation.x, t.transform.translation.y, yaw)
        return None

    try:
        say('WAITING FOR NAVIGATION')
        nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        nav.wait_for_server()
        if p['wait_for_survey']:
            survey = {'done': None}
            node.create_subscription(
                Bool, SURVEY_TOPIC, lambda m: survey.__setitem__('done', m.data), latched
            )
            say('SURVEYING START AREA')
            while rclpy.ok() and survey['done'] is None:
                rclpy.spin_once(node, timeout_sec=0.2)
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
            [to_map(park_pose(t, p['dump_reach']), fix) for t in dump_targets(p['dump_targets'])],
            to_map(p['home_pose'], fix),
            p['dig_seconds'],
            p['dump_seconds'],
        )
        log.info(f'mission: {p["cycles"]} dig-and-dump cycle(s), poses in {p["frame"]}')
        start = node.get_clock().now()
        for step in steps:
            say(step.state)
            if step.pose is None:
                sleep(step.seconds)
                continue
            for attempt in range(int(p['retries']) + 1):
                if drive(step.pose):
                    break
                if attempt < p['retries']:
                    say(f'{step.state} - RETRYING')
            else:
                say(f'FAILED: {step.state}')
                rclpy.spin(node)  # keep the latched state up
                return
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        say(f'MISSION COMPLETE in {elapsed:.0f} s')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
