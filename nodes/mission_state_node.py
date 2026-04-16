#!/usr/bin/env python3
"""
Mission State Node — Autonomy state machine for Lunabotics competition.

State machine
-------------
  TELEOP → (arm) → READY → (start_excavation / start_deposition) → AUTONOMOUS
  AUTONOMOUS → (sequence completes) → COMPLETE
  AUTONOMOUS → (abort / failure) → FAILED → (auto after 5 s) → TELEOP
  FAILED     → (arm)             → READY  (operator can re-arm immediately)
  COMPLETE   → (acknowledge)     → TELEOP
  ESTOP      → (acknowledge)     → TELEOP

FAILED auto-recovery
--------------------
When a sequence fails, the state machine holds FAILED for 5 seconds so the
operator sees it on the dashboard, then automatically returns to TELEOP.
This is purely timer-driven, implemented in _publish_state() — there is no
external signal required. The operator can bypass the timer by publishing
`arm` to /autonomy_command, which jumps straight to READY.

E-STOP latching
---------------
_on_estop() only fires on the rising edge of /emergency_stop. Releasing the
E-STOP does NOT auto-clear the state — the operator must publish
`acknowledge` to /autonomy_command. This is intentional: an accidental
momentary E-STOP must not silently re-enable autonomy.

Excavation sequence (runs in background thread)
  1. Publish +1.0 to lift mux autonomy input for lift_extend_s  — lower bucket
  2. Navigate to dig_pose                                        — drive to scoop
  3. Publish -1.0 to lift mux autonomy input for lift_retract_s — lift bucket
  4. Navigate to home_pose                                       — return home

Deposition sequence (runs in background thread)
  1. Navigate to hopper_pose                                     — drive to bin
  2. Publish +1.0 to tilt mux autonomy input for tilt_extend_s  — tilt to dump
  3. Sleep dump_wait_s                                           — let material fall
  4. Publish -1.0 to tilt mux autonomy input for tilt_retract_s — level bucket
  5. Navigate to home_pose                                       — return home

Actuator commands go through the actuator_mux_node at autonomy priority (1).
Teleop (priority 10) and GUI (priority 5) can override at any time.

Localization gate
-----------------
start_excavation / start_deposition are rejected unless /localization_status is
LOCALIZED (published by apriltag_localizer_node). Set require_localization:=false
to bypass during bench testing without AprilTags.

Sequence cooldown
-----------------
After an AUTONOMOUS sequence ends (either COMPLETE or FAILED transition from
AUTONOMOUS), the arm command is blocked for arm_cooldown_s seconds. This prevents
immediately re-running the linear actuators before they have cooled down. The
remaining cooldown is logged whenever arm is rejected.

Publishes:
  /autonomy_state                 (std_msgs/String) at 5 Hz
  /autonomy_cycle                 (std_msgs/Int32)
  /bucket/lift_mux/input/autonomy (std_msgs/Float64) during sequences
  /bucket/tilt_mux/input/autonomy (std_msgs/Float64) during sequences

Subscribes:
  /autonomy_command    (std_msgs/String) — arm | start_excavation | start_deposition | abort | acknowledge
  /emergency_stop      (std_msgs/Bool)
  /localization_status (std_msgs/String) — UNLOCALIZED | LOCALIZED | STALE

Parameters
----------
  dig_x, dig_y               — dig zone pose in map frame (default 4.0, 2.5)
  hopper_x, hopper_y         — ISRU hopper pose in map frame (default 0.5, 2.5)
  home_x, home_y             — home pose in map frame (default 0.3, 2.5)
  nav_timeout_s              — seconds before nav goal is cancelled (default 120.0)
  dump_wait_s                — seconds to wait while dumping (default 3.0)
  lift_extend_s              — seconds to publish lift extend command (default 18.0)
  lift_retract_s             — seconds to publish lift retract command (default 18.0)
  tilt_extend_s              — seconds to publish tilt extend command (default 20.0)
  tilt_retract_s             — seconds to publish tilt retract command (default 20.0)
  require_localization       — gate sequences on LOCALIZED status (default True)
  arm_cooldown_s             — seconds to block re-arm after a sequence ends (default 15.0)
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32, Bool, Float64
from geometry_msgs.msg import PoseStamped

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
        self.declare_parameter('nav_timeout_s', 120.0)
        self.declare_parameter('dump_wait_s',     3.0)
        # Actuator run durations — must match max_continuous_run_s in
        # bucket_actuators.yaml (18 s lift, 20 s tilt). Commands are published
        # at 10 Hz to /bucket/{lift,tilt}_mux/input/autonomy for this duration.
        self.declare_parameter('lift_extend_s',  18.0)
        self.declare_parameter('lift_retract_s', 18.0)
        self.declare_parameter('tilt_extend_s',  20.0)
        self.declare_parameter('tilt_retract_s', 20.0)
        self.declare_parameter('require_localization', True)
        self.declare_parameter('arm_cooldown_s', 15.0)

        # ── State ─────────────────────────────────────────────────────────────
        self._state: str          = TELEOP
        self._mode:  str | None   = None     # 'excavation' | 'deposition'
        self._cycle: int          = 0
        self._failed_time: float | None = None
        self._abort_event = threading.Event()
        self._localization_status: str     = 'UNLOCALIZED'
        self._last_auto_end_time: float | None = None
        self._sequence_thread: threading.Thread | None = None
        self._current_goal_handle = None      # holds Nav2 goal handle for cancel

        # ── Nav2 action client ────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group,
        )

        # ── Actuator command publishers (routed through actuator_mux_node) ────
        # Commands arrive at the mux at priority 1 (autonomy).
        # Teleop (10) and GUI (5) override when active.
        self._lift_pub = self.create_publisher(
            Float64, '/bucket/lift_mux/input/autonomy', 10,
            callback_group=self._cb_group,
        )
        self._tilt_pub = self.create_publisher(
            Float64, '/bucket/tilt_mux/input/autonomy', 10,
            callback_group=self._cb_group,
        )

        # ── Publishers / subscribers ──────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/autonomy_state', 10)
        self._cycle_pub = self.create_publisher(Int32,  '/autonomy_cycle', 10)

        self.create_subscription(String, '/autonomy_command',    self._on_command,             10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool,   '/emergency_stop',       self._on_estop,               10,
                                 callback_group=self._cb_group)
        self.create_subscription(String, '/localization_status',  self._on_localization_status, 10,
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
        if old == AUTONOMOUS:
            self._last_auto_end_time = time.monotonic()
        self._state = new_state
        self._failed_time = time.monotonic() if new_state == FAILED else None
        msg = f'State: {old} → {new_state}'
        if reason:
            msg += f'  ({reason})'
        self.get_logger().info(msg)

    def _check_cooldown(self) -> bool:
        """Return True if the arm cooldown has elapsed (or never ran)."""
        cooldown = self.get_parameter('arm_cooldown_s').value
        if self._last_auto_end_time is None:
            return True
        elapsed = time.monotonic() - self._last_auto_end_time
        if elapsed >= cooldown:
            return True
        remaining = cooldown - elapsed
        self.get_logger().warn(
            f'Arm rejected — actuator cooldown active: {remaining:.1f} s remaining '
            f'(arm_cooldown_s={cooldown:.1f})'
        )
        return False

    def _check_localization(self, cmd: str) -> bool:
        """Return True if localization gate passes (or is disabled)."""
        if not self.get_parameter('require_localization').value:
            return True
        if self._localization_status == 'LOCALIZED':
            return True
        self.get_logger().warn(
            f'Command "{cmd}" rejected — localization status is '
            f'"{self._localization_status}" (need LOCALIZED). '
            f'Set require_localization:=false to override.'
        )
        return False

    # ── Actuator command helper (runs from sequence thread) ───────────────────

    def _run_actuator(self, pub, value: float, duration_s: float, name: str) -> bool:
        """Publish a Float64 actuator command at 10 Hz for duration_s seconds.

        Commands go to the actuator_mux_node autonomy input (priority 1).
        Teleop (10) and GUI (5) can override at any time while this runs.
        Returns True on completion, False if abort was signalled.
        """
        self.get_logger().info(
            f'Actuator "{name}": publishing {value:+.1f} for {duration_s:.1f} s'
        )
        msg  = Float64(data=value)
        stop = Float64(data=0.0)
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if self._abort_event.is_set():
                pub.publish(stop)
                self.get_logger().info(f'Actuator "{name}": aborted')
                return False
            pub.publish(msg)
            time.sleep(0.1)
        pub.publish(stop)
        self.get_logger().info(f'Actuator "{name}": complete')
        return True

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
            if not self._run_actuator(self._lift_pub, 1.0,
                                      self.get_parameter('lift_extend_s').value,
                                      'lift extend'):
                raise RuntimeError('Lift extend aborted')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after lift extend')

            self.get_logger().info('Excavation sequence: step 2 — navigate to dig zone')
            if not self._navigate(self._make_pose(dig_x, dig_y), 'dig zone'):
                raise RuntimeError('Navigation to dig zone failed')

            if self._abort_event.is_set():
                raise RuntimeError('Aborted after navigate to dig')

            self.get_logger().info('Excavation sequence: step 3 — retract lift')
            if not self._run_actuator(self._lift_pub, -1.0,
                                      self.get_parameter('lift_retract_s').value,
                                      'lift retract'):
                raise RuntimeError('Lift retract aborted')

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
            if not self._run_actuator(self._tilt_pub, 1.0,
                                      self.get_parameter('tilt_extend_s').value,
                                      'tilt extend'):
                raise RuntimeError('Tilt extend aborted')

            self.get_logger().info(f'Deposition sequence: step 3 — waiting {dump_wait}s')
            for _ in range(int(dump_wait / 0.1)):
                if self._abort_event.is_set():
                    raise RuntimeError('Aborted during dump wait')
                time.sleep(0.1)

            self.get_logger().info('Deposition sequence: step 4 — retract tilt')
            if not self._run_actuator(self._tilt_pub, -1.0,
                                      self.get_parameter('tilt_retract_s').value,
                                      'tilt retract'):
                raise RuntimeError('Tilt retract aborted')

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

        # NOTE: this state machine is the source of truth for /autonomy_state.
        # The dashboard listens for the strings produced here; keep them in
        # sync with STATE_LABELS in dashboard/dashboard.html. AUTONOMOUS is
        # deliberately not in STATE_LABELS because the dashboard renders it
        # with a dynamic "Cycle X of 2" suffix.
        if self._state == TELEOP and cmd == 'arm':
            if not self._check_cooldown():
                return
            self._transition(READY, 'arm command')

        # Allow re-arming immediately from FAILED so the operator does not
        # have to wait out the 5 s auto-recovery timer after a bad sequence,
        # but still enforce the actuator cooldown.
        elif self._state == FAILED and cmd == 'arm':
            if not self._check_cooldown():
                return
            self._transition(READY, 'arm command (re-arm from FAILED)')

        elif self._state == READY and cmd == 'start_excavation':
            if not self._check_localization('start_excavation'):
                return
            self._mode = 'excavation'
            self._transition(AUTONOMOUS, 'excavation confirmed')
            self._start_sequence('excavation')

        elif self._state == READY and cmd == 'start_deposition':
            if not self._check_localization('start_deposition'):
                return
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

    def _on_localization_status(self, msg: String) -> None:
        self._localization_status = msg.data.strip().upper()

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
