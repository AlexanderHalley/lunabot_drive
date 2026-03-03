#!/usr/bin/env python3
"""
Bucket actuator driver node for Lunabot rover.

Drives Firgelli Super Duty linear actuators via BTS7960 H-bridge motor drivers
using Raspberry Pi 5 GPIO (lgpio). Falls back to mock/simulation mode when
lgpio is unavailable (e.g., MCC laptop during development/CI).

Two instances are launched: bucket/lift and bucket/tilt (see bucket_bringup.launch.py).
"""

import threading
import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState

# --- GPIO import with mock fallback ---
try:
    import lgpio
    _LGPIO_AVAILABLE = True
except ImportError:
    lgpio = None
    _LGPIO_AVAILABLE = False


class Direction(Enum):
    EXTEND = "extend"
    RETRACT = "retract"
    STOPPED = "stopped"


class MockGPIO:
    """Simulates lgpio calls for off-Pi development and testing."""

    def gpiochip_open(self, chip):
        return 0

    def gpiochip_close(self, h):
        pass

    def gpio_claim_output(self, h, gpio, val=0):
        pass

    def gpio_claim_input(self, h, gpio, lflags=0):
        pass

    def gpio_write(self, h, gpio, val):
        pass

    def gpio_read(self, h, gpio):
        return 0

    def tx_pwm(self, h, gpio, freq, duty, pulse_offset=0, pulse_cycles=0):
        pass

    def callback(self, h, gpio, edge, func):
        return object()  # dummy callback handle

    def callback_cancel(self, cb):
        pass

    RISING_EDGE = 1
    SET_PULL_UP = 2


class ActuatorDriverNode(Node):

    def __init__(self):
        super().__init__("actuator_driver")

        self._start_time = time.time()
        self._lock = threading.Lock()
        self._cb_group = ReentrantCallbackGroup()

        # --- Declare parameters ---
        self.declare_parameter("actuator_name", "lift")
        self.declare_parameter("rpwm_gpio", 12)
        self.declare_parameter("lpwm_gpio", 18)
        self.declare_parameter("en_gpio", 5)
        self.declare_parameter("hall_a_gpio", 22)
        self.declare_parameter("hall_b_gpio", 23)
        self.declare_parameter("stroke_mm", 203.2)
        self.declare_parameter("pulses_per_mm", 17.4)
        self.declare_parameter("deadband_mm", 1.0)
        self.declare_parameter("pwm_frequency", 10000)
        self.declare_parameter("max_speed_pct", 100)
        self.declare_parameter("approach_distance_mm", 20.0)
        self.declare_parameter("min_speed_pct", 30)
        self.declare_parameter("watchdog_timeout_s", 2.0)
        self.declare_parameter("home_on_startup", True)
        self.declare_parameter("max_continuous_run_s", 15.0)
        self.declare_parameter("cooldown_time_s", 45.0)
        self.declare_parameter("joint_name", "bucket_lift_joint")
        self.declare_parameter("position_to_joint_scale", 0.001)
        self.declare_parameter("position_to_joint_offset", 0.0)
        self.declare_parameter("invert_direction", False)

        # --- Load parameters ---
        self._actuator_name = self.get_parameter("actuator_name").value
        self._rpwm_gpio = self.get_parameter("rpwm_gpio").value
        self._lpwm_gpio = self.get_parameter("lpwm_gpio").value
        self._en_gpio = self.get_parameter("en_gpio").value
        self._hall_a_gpio = self.get_parameter("hall_a_gpio").value
        self._hall_b_gpio = self.get_parameter("hall_b_gpio").value
        self._stroke_mm = self.get_parameter("stroke_mm").value
        self._pulses_per_mm = self.get_parameter("pulses_per_mm").value
        self._deadband_mm = self.get_parameter("deadband_mm").value
        self._pwm_frequency = self.get_parameter("pwm_frequency").value
        self._max_speed_pct = self.get_parameter("max_speed_pct").value
        self._approach_distance_mm = self.get_parameter("approach_distance_mm").value
        self._min_speed_pct = self.get_parameter("min_speed_pct").value
        self._watchdog_timeout_s = self.get_parameter("watchdog_timeout_s").value
        self._home_on_startup = self.get_parameter("home_on_startup").value
        self._max_continuous_run_s = self.get_parameter("max_continuous_run_s").value
        self._cooldown_time_s = self.get_parameter("cooldown_time_s").value
        self._joint_name = self.get_parameter("joint_name").value
        self._pos_to_joint_scale = self.get_parameter("position_to_joint_scale").value
        self._pos_to_joint_offset = self.get_parameter("position_to_joint_offset").value
        self._invert_direction = self.get_parameter("invert_direction").value

        # --- State variables ---
        self._pulse_count: int = 0
        self._target_mm: float = 0.0
        self._direction: Direction = Direction.STOPPED
        self._current_speed_pct: int = 0
        self._homed: bool = False
        self._is_homing: bool = False
        self._last_command_time: float = time.time()

        # Duty cycle enforcement
        self._motor_run_start: Optional[float] = None
        self._cooldown_until: Optional[float] = None

        # Joint state velocity estimation
        self._prev_position_mm: float = 0.0
        self._prev_position_time: float = time.time()

        # --- GPIO setup ---
        self._mock_mode = not _LGPIO_AVAILABLE
        if self._mock_mode:
            self.get_logger().warn(
                f"lgpio not available — running in MOCK mode (no GPIO) for '{self._actuator_name}'"
            )
            self._gpio = MockGPIO()
        else:
            self._gpio = lgpio

        self._h = self._gpio.gpiochip_open(0)
        self._setup_gpio()

        # Mock mode: simulated position state
        self._mock_position_mm: float = 0.0
        self._mock_last_update: float = time.time()

        # --- ROS interfaces ---
        self._cmd_sub = self.create_subscription(
            Float64,
            "~/command",
            self._command_callback,
            10,
            callback_group=self._cb_group,
        )
        self._pos_pub = self.create_publisher(Float64, "~/position", 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, "~/status", 10)
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)

        self._home_srv = self.create_service(
            Trigger, "~/home", self._home_callback, callback_group=self._cb_group
        )
        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._stop_callback, callback_group=self._cb_group
        )

        # --- Timers ---
        self._control_timer = self.create_timer(
            0.02, self._control_loop, callback_group=self._cb_group  # 50 Hz
        )
        self._pos_timer = self.create_timer(
            0.05, self._publish_position, callback_group=self._cb_group  # 20 Hz
        )
        self._status_timer = self.create_timer(
            0.2, self._publish_status, callback_group=self._cb_group  # 5 Hz
        )

        self.get_logger().info(
            f"ActuatorDriverNode '{self._actuator_name}' initialized "
            f"({'MOCK' if self._mock_mode else 'REAL GPIO'})"
        )

        # Homing on startup (runs in background thread to not block init)
        if self._home_on_startup:
            threading.Thread(target=self._do_homing, daemon=True).start()

    # -----------------------------------------------------------------------
    # GPIO setup
    # -----------------------------------------------------------------------

    def _setup_gpio(self):
        if self._mock_mode:
            return

        # Outputs: initially LOW
        lgpio.gpio_claim_output(self._h, self._rpwm_gpio, 0)
        lgpio.gpio_claim_output(self._h, self._lpwm_gpio, 0)
        lgpio.gpio_claim_output(self._h, self._en_gpio, 0)

        # Inputs with pull-up for Hall sensors
        lgpio.gpio_claim_input(self._h, self._hall_a_gpio, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self._h, self._hall_b_gpio, lgpio.SET_PULL_UP)

        # Hall sensor interrupts
        self._hall_cb = lgpio.callback(
            self._h, self._hall_a_gpio, lgpio.RISING_EDGE, self._hall_pulse_callback
        )

        # Initialize PWM at 0% duty
        lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
        lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)

    # -----------------------------------------------------------------------
    # Hall sensor interrupt callback
    # -----------------------------------------------------------------------

    def _hall_pulse_callback(self, chip, gpio, level, tick):
        """Called on rising edge of Hall Channel A."""
        b_state = self._gpio.gpio_read(self._h, self._hall_b_gpio)
        with self._lock:
            if b_state == 0:
                self._pulse_count += 1  # extending
            else:
                self._pulse_count -= 1  # retracting

    # -----------------------------------------------------------------------
    # Position helpers
    # -----------------------------------------------------------------------

    def _get_position_mm(self) -> float:
        if self._mock_mode:
            return self._mock_position_mm
        with self._lock:
            raw = self._pulse_count / self._pulses_per_mm
        if raw < 0.0:
            self.get_logger().warn(
                f"[{self._actuator_name}] Position went negative ({raw:.2f} mm) — "
                "home position may be incorrect",
                throttle_duration_sec=5.0,
            )
        return max(0.0, min(raw, self._stroke_mm))

    def _update_mock_position(self):
        """Simulate linear actuator motion at 15 mm/s for mock mode."""
        now = time.time()
        dt = now - self._mock_last_update
        self._mock_last_update = now

        if self._is_homing:
            # Move toward 0 at homing speed
            self._mock_position_mm = max(0.0, self._mock_position_mm - 15.0 * dt)
            return

        error = self._target_mm - self._mock_position_mm
        if abs(error) <= self._deadband_mm:
            return

        direction = 1.0 if error > 0 else -1.0
        speed_mm_s = 15.0 * (self._current_speed_pct / 100.0)
        delta = direction * speed_mm_s * dt
        self._mock_position_mm = max(
            0.0, min(self._mock_position_mm + delta, self._stroke_mm)
        )

    # -----------------------------------------------------------------------
    # Motor control
    # -----------------------------------------------------------------------

    def _set_motor(self, direction: Direction, speed_pct: int):
        effective_dir = direction
        if self._invert_direction and direction != Direction.STOPPED:
            effective_dir = (
                Direction.RETRACT if direction == Direction.EXTEND else Direction.EXTEND
            )

        self._direction = direction
        self._current_speed_pct = speed_pct

        if self._mock_mode:
            return

        lgpio.gpio_write(self._h, self._en_gpio, 1)

        if effective_dir == Direction.EXTEND:
            lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, speed_pct)
            lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)
        elif effective_dir == Direction.RETRACT:
            lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
            lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, speed_pct)

    def _stop_motor(self):
        self._direction = Direction.STOPPED
        self._current_speed_pct = 0
        self._on_motor_stop()

        if self._mock_mode:
            return

        lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
        lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)

    # -----------------------------------------------------------------------
    # Duty cycle enforcement
    # -----------------------------------------------------------------------

    def _update_duty_cycle_tracker(self) -> bool:
        """Returns True if we should stop due to cooldown."""
        now = time.time()

        if self._cooldown_until and now < self._cooldown_until:
            return True  # still cooling down

        if self._motor_run_start is None:
            self._motor_run_start = now
            return False

        elapsed = now - self._motor_run_start
        if elapsed >= self._max_continuous_run_s:
            self.get_logger().warn(
                f"[{self._actuator_name}] Duty cycle limit reached ({elapsed:.1f}s). "
                f"Cooling down for {self._cooldown_time_s}s."
            )
            self._cooldown_until = now + self._cooldown_time_s
            self._motor_run_start = None
            return True

        return False

    def _on_motor_stop(self):
        self._motor_run_start = None

    def _in_cooldown(self) -> bool:
        if self._cooldown_until and time.time() < self._cooldown_until:
            return True
        return False

    # -----------------------------------------------------------------------
    # Control loop (50 Hz)
    # -----------------------------------------------------------------------

    def _control_loop(self):
        if self._is_homing:
            return  # homing thread drives the motor directly

        if self._mock_mode:
            self._update_mock_position()

        now = time.time()
        current_pos = self._get_position_mm()
        error = self._target_mm - current_pos

        # Watchdog
        if now - self._last_command_time > self._watchdog_timeout_s:
            if self._direction != Direction.STOPPED:
                self.get_logger().warn(
                    f"[{self._actuator_name}] Watchdog timeout — stopping motor.",
                    throttle_duration_sec=5.0,
                )
                self._stop_motor()
            return

        # Cooldown check
        if self._in_cooldown():
            if self._direction != Direction.STOPPED:
                self._stop_motor()
            return

        # At target?
        if abs(error) <= self._deadband_mm:
            if self._direction != Direction.STOPPED:
                self._stop_motor()
            return

        # Direction
        direction = Direction.EXTEND if error > 0 else Direction.RETRACT

        # Proportional speed ramp
        dist = abs(error)
        if dist < self._approach_distance_mm:
            speed_pct = int(
                self._min_speed_pct
                + (self._max_speed_pct - self._min_speed_pct)
                * (dist / self._approach_distance_mm)
            )
            speed_pct = max(self._min_speed_pct, min(speed_pct, self._max_speed_pct))
        else:
            speed_pct = self._max_speed_pct

        # Duty cycle enforcement
        if self._update_duty_cycle_tracker():
            self._stop_motor()
            return

        self._set_motor(direction, speed_pct)

    # -----------------------------------------------------------------------
    # Homing routine
    # -----------------------------------------------------------------------

    def _do_homing(self) -> bool:
        """Retract to limit switch, zero position. Returns True on success."""
        self._is_homing = True
        self.get_logger().info(
            f"[{self._actuator_name}] Homing: retracting to limit switch..."
        )

        # Retract at min speed
        self._set_motor(Direction.RETRACT, self._min_speed_pct)

        timeout = 30.0
        start = time.time()
        check_interval = 0.1  # 10 Hz

        last_pulse = None
        no_change_start = None

        while True:
            time.sleep(check_interval)
            now = time.time()

            if now - start > timeout:
                self.get_logger().error(
                    f"[{self._actuator_name}] Homing timed out after {timeout}s."
                )
                self._stop_motor()
                self._is_homing = False
                return False

            with self._lock:
                current_pulse = self._pulse_count

            if self._mock_mode:
                # Mock: homing complete when position reaches 0
                self._update_mock_position()
                if self._mock_position_mm <= 0.01:
                    break
                last_pulse = current_pulse
                no_change_start = None
                continue

            if last_pulse is None:
                last_pulse = current_pulse
                no_change_start = now
                continue

            if current_pulse != last_pulse:
                last_pulse = current_pulse
                no_change_start = now
            else:
                # Pulse count not changing
                if now - no_change_start >= 0.5:
                    # Limit switch reached
                    break

        self._stop_motor()

        with self._lock:
            self._pulse_count = 0

        self._mock_position_mm = 0.0
        self._target_mm = 0.0
        self._homed = True
        self._last_command_time = time.time()  # reset watchdog after homing
        self._is_homing = False

        self.get_logger().info(
            f"[{self._actuator_name}] Homing complete. Position zeroed."
        )
        return True

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------

    def _command_callback(self, msg: Float64):
        target = float(msg.data)
        target = max(0.0, min(target, self._stroke_mm))
        self._target_mm = target
        self._last_command_time = time.time()

    def _home_callback(self, request, response):
        success = self._do_homing()
        response.success = success
        response.message = (
            "Homing complete." if success else "Homing failed — see node logs."
        )
        return response

    def _stop_callback(self, request, response):
        self._stop_motor()
        self._target_mm = self._get_position_mm()  # hold current position
        response.success = True
        response.message = f"[{self._actuator_name}] Actuator stopped."
        return response

    # -----------------------------------------------------------------------
    # Publishers
    # -----------------------------------------------------------------------

    def _publish_position(self):
        pos = self._get_position_mm()

        # Position
        msg = Float64()
        msg.data = pos
        self._pos_pub.publish(msg)

        # Joint state
        now = time.time()
        dt = now - self._prev_position_time
        if dt > 0:
            velocity = ((pos - self._prev_position_mm) * self._pos_to_joint_scale) / dt
        else:
            velocity = 0.0
        self._prev_position_mm = pos
        self._prev_position_time = now

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self._joint_name]
        js.position = [pos * self._pos_to_joint_scale + self._pos_to_joint_offset]
        js.velocity = [velocity]
        js.effort = [0.0]
        self._joint_pub.publish(js)

    def _publish_status(self):
        pos = self._get_position_mm()
        error = self._target_mm - pos
        uptime = time.time() - self._start_time

        in_cooldown = self._in_cooldown()
        moving = self._direction != Direction.STOPPED

        if in_cooldown:
            level = DiagnosticStatus.WARN
            message = "Cooling down"
        elif self._is_homing:
            level = DiagnosticStatus.OK
            message = "Homing..."
        elif not self._homed:
            level = DiagnosticStatus.WARN
            message = "Not homed"
        elif moving:
            level = DiagnosticStatus.OK
            message = "Moving to target"
        else:
            level = DiagnosticStatus.OK
            message = "At position"

        status = DiagnosticStatus()
        status.name = f"bucket_{self._actuator_name}_driver"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key="position_mm", value=f"{pos:.2f}"),
            KeyValue(key="target_mm", value=f"{self._target_mm:.2f}"),
            KeyValue(key="error_mm", value=f"{error:.2f}"),
            KeyValue(key="moving", value=str(moving).lower()),
            KeyValue(key="speed_pct", value=str(self._current_speed_pct)),
            KeyValue(key="direction", value=self._direction.value),
            KeyValue(key="homed", value=str(self._homed).lower()),
            KeyValue(key="in_cooldown", value=str(in_cooldown).lower()),
            KeyValue(key="pulse_count", value=str(self._pulse_count)),
            KeyValue(key="uptime_s", value=f"{uptime:.1f}"),
        ]
        self._status_pub.publish(status)

    # -----------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info(
            f"[{self._actuator_name}] Shutting down actuator driver..."
        )
        self._stop_motor()

        if not self._mock_mode:
            lgpio.gpio_write(self._h, self._en_gpio, 0)
            try:
                self._hall_cb.cancel()
            except Exception:
                pass
            lgpio.gpiochip_close(self._h)

        self.get_logger().info(
            f"[{self._actuator_name}] Actuator driver shut down cleanly."
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorDriverNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
