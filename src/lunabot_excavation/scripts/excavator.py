#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavation server: dig and dump as actions, whatever moves the drum.

    ros2 run lunabot_excavation excavator.py --ros-args \
        --params-file $(ros2 pkg prefix lunabot_excavation)/share/lunabot_excavation/config/\
excavator.yaml

Serves (lunabot_msgs):
  /excavation/excavate  Excavate  lower the drum, dig, raise it
  /excavation/deposit   Deposit   tip the hopper, pour, bring it back
  /excavation/stow      std_srvs/Trigger: stop whatever it is doing and stow
Publishes:
  /excavation/status        ExcavatorStatus, 10 Hz
  /excavation/joint_states  sensor_msgs/JointState: excavator_arm, excavator_drum,
                            excavator_hopper (arm and hopper as 0..1 strokes)
  /excavation/fill          std_msgs/Float32, the hopper fill, 0..1
The last two are plain types for consumers that cannot load lunabot_msgs --
Isaac's bundled ROS, which draws the stand-in excavator from them.
Listens to /e_stop (std_msgs/Bool): engaged, everything holds where it is,
the running goal ends E_STOP, and new goals are refused until released.

A cancelled goal HOLDS (it does not stow), so a paused mission can send it
again with the time it had left. Every job begins by clearing the other
mechanism -- a dig first returns the hopper, a dump first raises the drum --
and every job that ends normally ends stowed, so a finished goal means the
rover may drive.

Backends: `model` is the only one -- a kinematic stand-in (strokes move at a
fixed rate, the hopper fills at a fixed rate while the drum digs, empties
while tipped) that drives nothing. It is what runs in Isaac and, until the
drum and hopper have drivers, on the rover: the mission then digs for its
time limit exactly as the old timed stop did, but through the interface the
driver will serve. A driver implements Backend (read / command / update).

Everything above main() is plain Python, tested without ROS.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

STATUS_TOPIC = '/excavation/status'
JOINTS_TOPIC = '/excavation/joint_states'
FILL_TOPIC = '/excavation/fill'
E_STOP_TOPIC = '/e_stop'
EXCAVATE_ACTION = '/excavation/excavate'
DEPOSIT_ACTION = '/excavation/deposit'
STOW_SERVICE = '/excavation/stow'
JOINT_NAMES = ('excavator_arm', 'excavator_drum', 'excavator_hopper')

# ExcavatorStatus.state, by name (the message's constants, kept equal: tested).
STATES = ('STOWED', 'HOLDING', 'EXCAVATING', 'DEPOSITING', 'STOWING', 'FAULT')
# Job outcomes. Excavate/Deposit results carry them as their uint8 constants.
FULL, EMPTY, TIME_LIMIT, CANCELED, E_STOP, FAULT = (
    'FULL',
    'EMPTY',
    'TIME_LIMIT',
    'CANCELED',
    'E_STOP',
    'FAULT',
)
# A stroke within this of its end counts as there.
AT = 0.02


@dataclass(frozen=True)
class Reading:
    """What the backend says the excavator is doing. NaN = unknown."""

    arm: float  # 0 stowed .. 1 digging depth
    drum_velocity: float  # rad/s, positive digs
    hopper: float  # 0 flat .. 1 tipped
    fill: float  # 0 empty .. 1 full
    fill_measured: bool = False
    fault: str = ''


@dataclass(frozen=True)
class Command:
    """Where the backend should take the strokes, and how fast to turn the drum."""

    arm: float
    drum_velocity: float
    hopper: float


def hold(r: Reading) -> Command:
    """Stay exactly where it is, drum stopped."""
    arm = 0.0 if math.isnan(r.arm) else r.arm
    hopper = 0.0 if math.isnan(r.hopper) else r.hopper
    return Command(arm, 0.0, hopper)


def _at(value: float, target: float) -> bool:
    return not math.isnan(value) and abs(value - target) <= AT


def stowed(r: Reading) -> bool:
    return _at(r.arm, 0.0) and _at(r.hopper, 0.0) and abs(r.drum_velocity or 0.0) < 1e-6


class ModelBackend:
    """A kinematic excavator with no hardware: the stand-in until there are drivers.

    The strokes move toward their targets at a fixed rate; the drum turns as
    commanded. The hopper fills at `fill_rate` per second while the drum is at
    depth and turning, and empties at `empty_rate` once tipped past
    `pour_from`. Its fill is an estimate, so it reports fill_measured False
    unless told otherwise -- goals then end on their time limit, as they will
    on a rover with no load sensing.
    """

    name = 'model'

    def __init__(
        self,
        arm_seconds=2.4,
        hopper_seconds=1.4,
        fill_rate=0.12,
        empty_rate=0.5,
        pour_from=0.7,
        fill_measured=False,
    ):
        self.arm_rate = 1.0 / arm_seconds
        self.hopper_rate = 1.0 / hopper_seconds
        self.fill_rate = fill_rate
        self.empty_rate = empty_rate
        self.pour_from = pour_from
        self.fill_measured = bool(fill_measured)
        self.arm = self.hopper = self.drum = self.fill = 0.0
        self.target = Command(0.0, 0.0, 0.0)
        self.stuck = ''  # tests: an actuator that will not move ('arm' / 'hopper')

    def read(self) -> Reading:
        return Reading(self.arm, self.drum, self.hopper, self.fill, self.fill_measured)

    def command(self, cmd: Command) -> None:
        self.target = cmd

    def update(self, dt: float) -> None:
        def towards(value, target, step):
            return (
                target
                if abs(target - value) <= step
                else value + math.copysign(step, target - value)
            )

        if self.stuck != 'arm':
            self.arm = towards(self.arm, self.target.arm, self.arm_rate * dt)
        if self.stuck != 'hopper':
            self.hopper = towards(self.hopper, self.target.hopper, self.hopper_rate * dt)
        self.drum = self.target.drum_velocity
        if self.arm >= 1.0 - AT and self.drum > 0.0:
            self.fill = min(1.0, self.fill + self.fill_rate * dt)
        if self.hopper >= self.pour_from:
            self.fill = max(0.0, self.fill - self.empty_rate * dt)


@dataclass
class Job:
    """One goal: a list of phases run in turn, each until its condition holds.

    `seconds` counts only the working phase (drum digging, hopper tipped),
    which is what the goal's max_seconds limits.
    """

    kind: str  # excavate, deposit, stow
    max_seconds: float = math.inf
    target_fill: float = 1.0
    dig_speed: float = 4.2
    move_timeout: float = 10.0
    phase_index: int = 0
    phase_time: float = 0.0
    seconds: float = 0.0
    reason: str = ''  # why the working phase ended: FULL / EMPTY / TIME_LIMIT
    outcome: str = ''  # set once the job is over
    message: str = ''
    fill: float = math.nan
    phases: list = field(default_factory=list)

    def __post_init__(self):
        self.phases = {
            'excavate': ['RETURNING', 'LOWERING', 'DIGGING', 'RAISING'],
            'deposit': ['RAISING', 'TIPPING', 'POURING', 'RETURNING'],
            'stow': ['STOWING'],
        }[self.kind]
        if self.max_seconds <= 0.0:
            # No working time left (a resumed goal that was cancelled while
            # raising, say): finish -- do not lower the drum again to dig 0 s.
            self.phases = [ph for ph in self.phases if ph in ('RETURNING', 'RAISING')]
            self.reason = TIME_LIMIT

    @property
    def phase(self) -> str:
        return self.phases[self.phase_index] if not self.outcome else ''

    @property
    def done(self) -> bool:
        return bool(self.outcome)

    def end(self, outcome: str, message: str = '') -> None:
        if not self.outcome:
            self.outcome, self.message = outcome, message or outcome.lower().replace('_', ' ')

    def step(self, r: Reading, dt: float) -> Command:
        """Advance by dt with the latest reading; return what to command."""
        self.fill = r.fill
        while not self.outcome:
            phase = self.phases[self.phase_index]
            cmd, finished = self._phase(phase, r, dt)
            if not finished:
                self.phase_time += dt
                if phase not in ('DIGGING', 'POURING') and self.phase_time > self.move_timeout:
                    self.end(FAULT, f'{phase} took over {self.move_timeout:.0f} s')
                    return hold(r)
                return cmd
            self.phase_index += 1
            self.phase_time = 0.0
            dt = 0.0  # the next phase starts from this same reading
            if self.phase_index == len(self.phases):
                self.end(self.reason or ('STOWED' if self.kind == 'stow' else TIME_LIMIT))
        return hold(r)

    def _phase(self, phase: str, r: Reading, dt: float):
        """Return (command, finished) for one phase."""
        if phase == 'STOWING':
            return Command(0.0, 0.0, 0.0), _at(r.arm, 0.0) and _at(r.hopper, 0.0)
        if phase == 'RETURNING':  # hopper flat, drum where it is
            return Command(_keep(r.arm), 0.0, 0.0), _at(r.hopper, 0.0)
        if phase == 'RAISING':  # drum up, hopper where it is
            return Command(0.0, 0.0, _keep(r.hopper)), _at(r.arm, 0.0)
        if phase == 'LOWERING':
            return Command(1.0, 0.0, 0.0), _at(r.arm, 1.0)
        if phase == 'TIPPING':
            return Command(0.0, 0.0, 1.0), _at(r.hopper, 1.0)
        if phase == 'DIGGING':
            if r.fill_measured and not math.isnan(r.fill) and r.fill >= self.target_fill:
                self.reason = FULL
                return Command(1.0, 0.0, 0.0), True
            if self.seconds >= self.max_seconds:
                self.reason = TIME_LIMIT
                return Command(1.0, 0.0, 0.0), True
            self.seconds += dt
            return Command(1.0, self.dig_speed, 0.0), False
        if phase == 'POURING':
            if r.fill_measured and not math.isnan(r.fill) and r.fill <= AT:
                self.reason = EMPTY
                return Command(0.0, 0.0, 1.0), True
            if self.seconds >= self.max_seconds:
                self.reason = TIME_LIMIT
                return Command(0.0, 0.0, 1.0), True
            self.seconds += dt
            return Command(0.0, 0.0, 1.0), False
        raise ValueError(phase)


def _keep(value: float) -> float:
    return 0.0 if math.isnan(value) else value


class Excavator:
    """One job at a time over a backend, plus the e-stop. Pure: tested without ROS.

    start() / cancel() / stow() / set_e_stop() are the commands; tick(dt)
    runs the backend and the job, and is the only thing that moves anything.
    """

    def __init__(self, backend, dig_speed=4.2, move_timeout=10.0):
        self.backend = backend
        self.dig_speed = dig_speed
        self.move_timeout = move_timeout
        self.job: Job | None = None
        self.e_stop = False
        self.fault = ''
        self.reading = backend.read()

    @property
    def busy(self) -> bool:
        return self.job is not None and not self.job.done

    def can_start(self):
        """Return (ok, why not): the same check start() makes, for a goal callback."""
        if self.e_stop:
            return False, 'e-stop engaged'
        if self.busy:
            return False, f'busy: {self.job.kind}'
        return True, ''

    def start(self, kind: str, max_seconds: float = math.inf, target_fill: float = 1.0):
        """Begin a job; return it, or None if refused (see can_start)."""
        if kind == 'stow':
            return self.stow()
        if not self.can_start()[0]:
            return None
        self.fault = ''
        self.job = Job(
            kind,
            max_seconds=max(0.0, max_seconds),
            target_fill=target_fill,
            dig_speed=self.dig_speed,
            move_timeout=self.move_timeout,
        )
        return self.job

    def stow(self):
        """Stop the running job (it ends CANCELED) and stow. Refused under the e-stop."""
        if self.e_stop:
            return None
        self.cancel()
        self.fault = ''
        self.job = Job('stow', move_timeout=self.move_timeout)
        return self.job

    def cancel(self) -> None:
        if self.busy:
            self.job.end(CANCELED)
            self.backend.command(hold(self.reading))

    def set_e_stop(self, engaged: bool) -> None:
        if engaged and self.busy:
            self.job.end(E_STOP, 'e-stop')
        if engaged:
            self.backend.command(hold(self.reading))
        self.e_stop = bool(engaged)

    def tick(self, dt: float) -> Reading:
        self.backend.update(dt)
        r = self.reading = self.backend.read()
        if r.fault and self.busy:
            self.job.end(FAULT, r.fault)
        if self.busy and not self.e_stop:
            self.backend.command(self.job.step(r, dt))
            if self.job.outcome == FAULT:
                self.fault = self.job.message
        else:
            self.backend.command(hold(r))
        self.fault = self.fault or r.fault
        return r

    def state(self) -> str:
        """Return ExcavatorStatus.state by name."""
        if self.fault:
            return 'FAULT'
        if self.busy and not self.e_stop:
            return {'excavate': 'EXCAVATING', 'deposit': 'DEPOSITING', 'stow': 'STOWING'}[
                self.job.kind
            ]
        return 'STOWED' if stowed(self.reading) else 'HOLDING'

    def phase(self) -> str:
        return self.job.phase if self.busy else ''


def make_backend(name: str, params: dict):
    """Return the backend called `name`, built from its parameters."""
    if name == 'model':
        return ModelBackend(**params)
    raise ValueError(f'unknown excavator backend {name!r}: the only one so far is "model"')


MODEL_PARAMS = {
    'arm_seconds': 2.4,
    'hopper_seconds': 1.4,
    'fill_rate': 0.12,
    'empty_rate': 0.5,
    'pour_from': 0.7,
    'fill_measured': False,
}


def main() -> None:
    import threading
    import time

    import rclpy
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32
    from std_srvs.srv import Trigger

    from lunabot_msgs.action import Deposit, Excavate
    from lunabot_msgs.msg import ExcavatorStatus

    rclpy.init()
    node = Node('excavator')
    log = node.get_logger()
    p = {
        name: node.declare_parameter(name, default).value
        for name, default in (
            ('backend', 'model'),
            ('rate', 20.0),
            ('dig_speed', 4.2),  # rad/s
            ('move_timeout', 10.0),  # s for one stroke before it is a FAULT
        )
    }
    model = {k: node.declare_parameter(f'model.{k}', v).value for k, v in MODEL_PARAMS.items()}
    backend = make_backend(p['backend'], model if p['backend'] == 'model' else {})
    core = Excavator(backend, dig_speed=p['dig_speed'], move_timeout=p['move_timeout'])
    lock = threading.Lock()
    group = ReentrantCallbackGroup()
    log.info(f'excavator: backend {backend.name}')

    status_pub = node.create_publisher(ExcavatorStatus, STATUS_TOPIC, 10)
    joints_pub = node.create_publisher(JointState, JOINTS_TOPIC, 10)
    fill_pub = node.create_publisher(Float32, FILL_TOPIC, 10)
    clock = {'last': None, 'n': 0, 'drum_angle': 0.0}

    def publish(r: Reading) -> None:
        now = node.get_clock().now().to_msg()
        s = ExcavatorStatus()
        s.header.stamp = now
        s.state = getattr(ExcavatorStatus, core.state())
        s.phase = core.phase()
        s.arm_position, s.drum_velocity, s.hopper_position = r.arm, r.drum_velocity, r.hopper
        s.fill, s.fill_measured = r.fill, r.fill_measured
        s.e_stop, s.backend, s.fault = core.e_stop, backend.name, core.fault
        status_pub.publish(s)
        j = JointState()
        j.header.stamp = now
        j.name = list(JOINT_NAMES)
        j.position = [r.arm, clock['drum_angle'], r.hopper]
        j.velocity = [math.nan, r.drum_velocity, math.nan]
        joints_pub.publish(j)
        fill_pub.publish(Float32(data=r.fill))

    def tick() -> None:
        # Node clock: sim time in Isaac, so a 10 s dig is 10 s on /clock.
        now = node.get_clock().now().nanoseconds * 1e-9
        dt = 0.0 if clock['last'] is None else min(max(now - clock['last'], 0.0), 0.5)
        clock['last'] = now
        with lock:
            before = core.state()
            r = core.tick(dt)
            clock['drum_angle'] = (clock['drum_angle'] + r.drum_velocity * dt) % math.tau
            after = core.state()
        if after != before:
            log.info(
                f'excavator: {before} -> {after}' + (f' ({core.fault})' if core.fault else '')
            )
        clock['n'] += 1
        if clock['n'] % max(1, round(p['rate'] / 10.0)) == 0:
            publish(r)

    node.create_timer(1.0 / p['rate'], tick, callback_group=group)

    def on_e_stop(msg) -> None:
        with lock:
            if bool(msg.data) != core.e_stop:
                log.warning(f'excavator: e-stop {"ENGAGED: holding" if msg.data else "released"}')
            core.set_e_stop(msg.data)

    # Default QoS, as twist_mux's lock and mission.py subscribe.
    node.create_subscription(Bool, E_STOP_TOPIC, on_e_stop, 10, callback_group=group)

    def serve_stow(_request, response):
        with lock:
            job = core.stow()
        response.success = job is not None
        response.message = 'stowing' if job else 'e-stop engaged'
        log.info(f'{STOW_SERVICE}: {response.message}')
        return response

    node.create_service(Trigger, STOW_SERVICE, serve_stow, callback_group=group)

    def goal_callback(_goal):
        with lock:
            ok, why = core.can_start()
        if not ok:
            log.warning(f'excavator: goal refused ({why})')
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def executor_for(kind, action_type):
        def execute(goal_handle):
            g = goal_handle.request
            with lock:
                job = core.start(
                    kind,
                    max_seconds=g.max_seconds,
                    target_fill=getattr(g, 'target_fill', 1.0),
                )
            result = action_type.Result()
            if job is None:  # the e-stop or another goal got in first
                result.outcome = action_type.Result.FAULT
                result.message = 'refused: ' + core.can_start()[1]
                goal_handle.abort()
                return result
            log.info(f'excavator: {kind} for up to {g.max_seconds:.1f} s')
            feedback = action_type.Feedback()
            while rclpy.ok() and not job.done:
                if goal_handle.is_cancel_requested:
                    with lock:
                        core.cancel()
                    break
                feedback.phase, feedback.fill, feedback.seconds = job.phase, job.fill, job.seconds
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.outcome = getattr(action_type.Result, job.outcome or CANCELED)
            result.fill, result.seconds, result.message = job.fill, job.seconds, job.message
            log.info(f'excavator: {kind} {job.outcome} after {job.seconds:.1f} s')
            if job.outcome == CANCELED:
                goal_handle.canceled()
            elif job.outcome in (E_STOP, FAULT):
                goal_handle.abort()
            else:
                goal_handle.succeed()
            return result

        return execute

    servers = [
        ActionServer(
            node,
            action_type,
            name,
            executor_for(kind, action_type),
            goal_callback=goal_callback,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
            callback_group=group,
        )
        for kind, action_type, name in (
            ('excavate', Excavate, EXCAVATE_ACTION),
            ('deposit', Deposit, DEPOSIT_ACTION),
        )
    ]
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
