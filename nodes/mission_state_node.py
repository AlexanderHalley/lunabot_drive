#!/usr/bin/env python3
"""
Mission State Node — Autonomy state machine for Lunabotics competition.

State machine
-------------
  TELEOP → (arm) → READY → (start_excavation / start_deposition) → AUTONOMOUS
  AUTONOMOUS → (sequence completes) → COMPLETE
  AUTONOMOUS → (abort / failure) → FAILED → (auto after 5 s) → TELEOP
  COMPLETE   → (acknowledge)     → TELEOP
  ESTOP      → (acknowledge)     → TELEOP

Excavation sequence (runs in background thread)
  1. Call /lift_actuator/extend  — lower bucket into regolith position
  2. Navigate to dig_pose        — drive forward to scoop material
  3. Call /lift_actuator/retract — lift bucket
  4. Navigate to home_pose       — return to start area

Deposition sequence (runs in background thread)
  1. Navigate to hopper_pose     — drive to ISRU bin
  2. Call /tilt_actuator/extend  — tilt bucket to dump
  3. Sleep dump_wait_s           — let material fall
  4. Call /tilt_actuator/retract — return bucket
  5. Navigate to home_pose       — return to start area

Publishes:
  /autonomy_state  (std_msgs/String) at 5 Hz
  /autonomy_cycle  (std_msgs/Int32)

Subscribes:
  /autonomy_command (std_msgs/String) — arm | start_excavation | start_deposition | abort | acknowledge
  /emergency_stop   (std_msgs/Bool)

Parameters
----------
  dig_x, dig_y               — dig zone pose in map frame (default 4.0, 2.5)
  hopper_x, hopper_y         — ISRU hopper pose in map frame (default 0.5, 2.5)
  home_x, home_y             — home pose in map frame (default 0.3, 2.5)
  nav_timeout_s              — seconds before nav goal is cancelled (default 120.0)
  service_timeout_s          — seconds to wait for actuator service (default 35.0)
  dump_wait_s                — seconds to wait while dumping (default 3.0)
  lift_actuator_ns           — namespace for lift actuator (default '/lift_actuator')
  tilt_actuator_ns           — namespace for tilt actuator (default '/tilt_actuator')
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── State constants ──────────────────────────────────────────────────────────
TELEOP     = 'TELEOP'
READY      = 'READY'
AUTONOMOUS = 'AUTONOMOUS'
COMPLETE   = 'COMPLETE'
FAILED     = 'FAILED'
ESTOP      = 'ESTOP'


class MissionStateNode(Node):

    def __init__(self):
        super().__init__('mission_state_node')

        self._cb_group = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('dig_x',    4.0)
        self.declare_parameter('dig_y',    2.5)
        self.declare_parameter('hopper_x', 0.5)
        self.declare_parameter('hopper_y', 2.5)
        self.declare_parameter('home_x',   0.3)
        self.declare_parameter('home_y',   2.5)
        self.declare_parameter('nav_timeout_s',     120.0)
        self.declare_parameter('service_timeout_s',  35.0)
        self.declare_parameter('dump_wait_s',          3.0)
        self.declare_parameter('lift_actuator_ns', '/bucket/lift/actuator_driver')
        self.declare_parameter('tilt_actuator_ns', '/bucket/tilt/actuator_driver')

        lift_ns = self.get_parameter('lift_actuator_ns').value
        tilt_ns = self.get_parameter('tilt_actuator_ns').value

        # ── State ─────────────────────────────────────────────────────────────
        self._state: str          = TELEOP
        self._mode:  str | None   = None     # 'excavation' | 'deposition'
        self._cycle: int          = 0
        self._failed_time: float | None = None
        self._abort_event = threading.Event()
        self._sequence_thread: threading.Thread | None = None
        self._current_goal_handle = None      # holds Nav2 goal handle for cancel

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        # ── Actuator service clients ───────────────────────────────────────────
        self._lift_extend  = self.create_client(Trigger, f'{lift_ns}/extend',  callback_group=self._cb_group)
        self._lift_retract = self.create_client(Trigger, f'{lift_ns}/retract', callback_group=self._cb_group)
        self._tilt_extend  = self.create_client(Trigger, f'{tilt_ns}/extend',  callback_group=self._cb_group)
        self._tilt_retract = self.create_client(Trigger, f'{tilt_ns}/retract', callback_group=self._cb_group)

        # ── Publishers / subscribers ──────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/autonomy_state', 10)
        self._cycle_pub = self.create_publisher(Int32,  '/autonomy_cycle', 10)

        self.create_subscription(String, '/autonomy_command', self._on_command, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool,   '/emergency_stop',   self._on_estop,   10,
                                 callback_group=self._cb_group)

        self.create_timer(0.2, self._publish_state, callback_group=self._cb_group)

        self.get_logger().info('Mission state node started — initial state: TELEOP')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _make_pose(self, x: float, y: float, frame: str = 'map') -> PoseStamped:
        p = PoseStamped()
        p.header.frame_id = frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = 0.0
        p.pose.orientation.w = 1.0
        return p

    def _transition(self, new_state: str, reason: str = '') -> None:
        old = self._state
        self._state = new_state
        self._failed_time = time.monotonic() if new_state == FAILED else None
        msg = f'State: {old} → {new_state}'
        if reason:
            msg += f'  ({reason})'
        self.get_logger().info(msg)

    # ── Service call helper (runs from sequence thread) ───────────────────────

    def _call_service(self, client, name: str) -> bool:
        """Call a Trigger service and return True on success. Polls until done."""
        timeout = self.get_parameter('service_timeout_s').value
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Service {name} not available')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._abort_event.is_set():
                return False
            if time.monotonic() > deadline:
                self.get_logger().error(f'Service {name} timed out')
                return False
            time.sleep(0.05)
        result = future.result()
        if not result.success:
            self.get_logger().warn(f'Service {name} returned failure: {result.message}')
        return result.success

    # ── Nav2 navigation helper (runs from sequence thread) ────────────────────

    def _navigate(self, pose: PoseStamped, description: str) -> bool:
        """Send NavigateToPose goal and block until complete or abort."""
        nav_timeout = self.get_parameter('nav_timeout_s').value

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(
            f'Navigating to {description} '
            f'({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})'
        )

        # Send goal async
        send_future = self._nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + nav_timeout

        # Wait for goal acceptance
        while not send_future.done():
            if self._abort_event.is_set():
                return False
            if time.monotonic() > deadline:
                self.get_logger().error(f'Navigation goal send timed out ({description})')
                return False
            time.sleep(0.05)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Navigation goal rejected ({description})')
            return False

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()

        # Wait for result
        while not result_future.done():
            if self._abort_event.is_set():
                self.get_logger().info(f'Aborting navigation ({description})')
                goal_handle.cancel_goal_async()
                self._current_goal_handle = None
                return False
            if time.monotonic() > deadline:
                self.get_logger().error(f'Navigation timed out ({description})')
                goal_handle.cancel_goal_async()
                self._current_goal_handle = None
                return False
            time.sleep(0.05)

        self._current_goal_handle = None
        result = result_future.result()
        success = (result.status == GoalStatus.STATUS_SUCCEEDED)
        if not success:
            self.get_logger().warn(
                f'Navigation to {description} ended with status {result.status}'
            )
        return success

    # ── Autonomy sequences (run in background threads) ────────────────────────

    def _excavation_sequence(self) -> None:
        """
        Full excavation cycle:
          extend lift → navigate to dig zone → retract lift → navigate home
        """
        dig_x    = self.get_parameter('dig_x').value
        dig_y    = self.get_parameter('dig_y').value
        home_x   = self.get_parameter('home_x').value
        home_y   = self.get_parameter('home_y').value

        try:
            self.get_logger().info('Excavation sequence: step 1 — extend lift')
            if not self._call_service(self._lift_extend, 'lift/extend'):
                raise RuntimeError('Lift extend failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after lift extend')

            self.get_logger().info('Excavation sequence: step 2 — navigate to dig zone')
            if not self._navigate(self._make_pose(dig_x, dig_y), 'dig zone'):
                raise RuntimeError('Navigation to dig zone failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after navigate to dig')

            self.get_logger().info('Excavation sequence: step 3 — retract lift')
            if not self._call_service(self._lift_retract, 'lift/retract'):
                raise RuntimeError('Lift retract failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after lift retract')

            self.get_logger().info('Excavation sequence: step 4 — navigate home')
            if not self._navigate(self._make_pose(home_x, home_y), 'home'):
                raise RuntimeError('Navigation home failed')

            self._transition(COMPLETE, 'excavation sequence finished')

        except RuntimeError as exc:
            if not self._abort_event.is_set():
                self.get_logger().error(f'Excavation sequence failed: {exc}')
            self._transition(FAILED, str(exc))

    def _deposition_sequence(self) -> None:
        """
        Full deposition cycle:
          navigate to hopper → tilt extend → wait → tilt retract → navigate home
        """
        hopper_x  = self.get_parameter('hopper_x').value
        hopper_y  = self.get_parameter('hopper_y').value
        home_x    = self.get_parameter('home_x').value
        home_y    = self.get_parameter('home_y').value
        dump_wait = self.get_parameter('dump_wait_s').value

        try:
            self.get_logger().info('Deposition sequence: step 1 — navigate to hopper')
            if not self._navigate(self._make_pose(hopper_x, hopper_y), 'hopper'):
                raise RuntimeError('Navigation to hopper failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after navigate to hopper')

            self.get_logger().info('Deposition sequence: step 2 — tilt bucket to dump')
            if not self._call_service(self._tilt_extend, 'tilt/extend'):
                raise RuntimeError('Tilt extend failed')

            self.get_logger().info(f'Deposition sequence: step 3 — waiting {dump_wait}s')
            for _ in range(int(dump_wait / 0.1)):
                if self._abort_event.is_set():
                    raise RuntimeError('Aborted during dump wait')
                time.sleep(0.1)

            self.get_logger().info('Deposition sequence: step 4 — retract tilt')
            if not self._call_service(self._tilt_retract, 'tilt/retract'):
                raise RuntimeError('Tilt retract failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after tilt retract')

            self.get_logger().info('Deposition sequence: step 5 — navigate home')
            if not self._navigate(self._make_pose(home_x, home_y), 'home'):
                raise RuntimeError('Navigation home failed')

            self._transition(COMPLETE, 'deposition sequence finished')

        except RuntimeError as exc:
            if not self._abort_event.is_set():
                self.get_logger().error(f'Deposition sequence failed: {exc}')
            self._transition(FAILED, str(exc))

    def _start_sequence(self, mode: str) -> None:
        """Kick off a sequence in a daemon thread."""
        self._abort_event.clear()
        target = self._excavation_sequence if mode == 'excavation' else self._deposition_sequence
        self._sequence_thread = threading.Thread(target=target, daemon=True)
        self._sequence_thread.start()

    def _cancel_sequence(self) -> None:
        """Signal abort and cancel any active nav goal."""
        self._abort_event.set()
        if self._current_goal_handle is not None:
            self._current_goal_handle.cancel_goal_async()
            self._current_goal_handle = None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _on_command(self, msg: String) -> None:
        cmd = msg.data.strip().lower()

        if self._state == TELEOP and cmd == 'arm':
            self._transition(READY, 'arm command')

        elif self._state == READY and cmd == 'start_excavation':
            self._mode = 'excavation'
            self._transition(AUTONOMOUS, 'excavation confirmed')
            self._start_sequence('excavation')

        elif self._state == READY and cmd == 'start_deposition':
            self._mode = 'deposition'
            self._transition(AUTONOMOUS, 'deposition confirmed')
            self._start_sequence('deposition')

        elif self._state == READY and cmd == 'abort':
            self._transition(TELEOP, 'abort from READY')

        elif self._state == AUTONOMOUS and cmd == 'abort':
            self._cancel_sequence()
            self._transition(FAILED, 'manual abort during AUTONOMOUS')

        elif self._state == COMPLETE and cmd == 'acknowledge':
            self._cycle += 1
            self._transition(TELEOP, f'cycle {self._cycle} acknowledged')

        elif self._state == ESTOP and cmd == 'acknowledge':
            self._transition(TELEOP, 'E-STOP acknowledged')

        else:
            self.get_logger().debug(f'Command "{cmd}" ignored in state {self._state}')

    def _on_estop(self, msg: Bool) -> None:
        if msg.data and self._state != ESTOP:
            self._cancel_sequence()
            self._transition(ESTOP, 'emergency stop received')

    # ── Timer: publish state + FAILED auto-recovery ───────────────────────────

    def _publish_state(self) -> None:
        if self._state == FAILED and self._failed_time is not None:
            if time.monotonic() - self._failed_time >= 5.0:
                self._transition(TELEOP, 'auto-recovery after 5 s in FAILED')

        self._state_pub.publish(String(data=self._state))
        self._cycle_pub.publish(Int32(data=self._cycle))


def main(args=None):
    rclpy.init(args=args)
    node = MissionStateNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
