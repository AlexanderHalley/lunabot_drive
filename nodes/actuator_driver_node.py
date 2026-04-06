#!/usr/bin/env python3
"""
Bucket actuator driver node for Lunabot rover.

Open-loop drive — no Hall sensors required.

Command topic: ~/command (Float64)
  +1.0 = full extend
  -1.0 = full retract
   0.0 = stop
  Values are clamped to [-1.0, 1.0]. Actual duty = abs(value) * max_speed_pct.

A 2-second watchdog stops the motor if no command arrives.
A duty-cycle limiter stops the motor after max_continuous_run_s and enforces
a cooldown_time_s pause to protect the actuator.
"""

import time
from enum import Enum
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

try:
    import lgpio
    _LGPIO_AVAILABLE = True
except ImportError:
    lgpio = None
    _LGPIO_AVAILABLE = False


class Direction(Enum):
    EXTEND  = "extend"
    RETRACT = "retract"
    STOPPED = "stopped"


class MockGPIO:
    def gpiochip_open(self, chip):      return 0
    def gpiochip_close(self, h):        pass
    def gpio_claim_output(self, h, gpio, val=0): pass
    def gpio_write(self, h, gpio, val): pass
    def tx_pwm(self, h, gpio, freq, duty, pulse_offset=0, pulse_cycles=0): pass


class ActuatorDriverNode(Node):

    def __init__(self):
        super().__init__("actuator_driver")

        self._start_time = time.time()
        self._cb_group = ReentrantCallbackGroup()

        # --- Parameters ---
        self.declare_parameter("actuator_name",       "lift")
        self.declare_parameter("rpwm_gpio",           12)
        self.declare_parameter("lpwm_gpio",           18)
        self.declare_parameter("en_gpio",             5)
        self.declare_parameter("pwm_frequency",       10000)
        self.declare_parameter("max_speed_pct",       100)
        self.declare_parameter("watchdog_timeout_s",  2.0)
        self.declare_parameter("max_continuous_run_s", 15.0)
        self.declare_parameter("cooldown_time_s",     45.0)
        self.declare_parameter("invert_direction",    False)

        self._actuator_name     = self.get_parameter("actuator_name").value
        self._rpwm_gpio         = self.get_parameter("rpwm_gpio").value
        self._lpwm_gpio         = self.get_parameter("lpwm_gpio").value
        self._en_gpio           = self.get_parameter("en_gpio").value
        self._pwm_frequency     = self.get_parameter("pwm_frequency").value
        self._max_speed_pct     = self.get_parameter("max_speed_pct").value
        self._watchdog_timeout_s = self.get_parameter("watchdog_timeout_s").value
        self._max_continuous_run_s = self.get_parameter("max_continuous_run_s").value
        self._cooldown_time_s   = self.get_parameter("cooldown_time_s").value
        self._invert_direction  = self.get_parameter("invert_direction").value

        # --- State ---
        self._drive_value: float      = 0.0   # last command, [-1, 1]
        self._direction: Direction    = Direction.STOPPED
        self._current_speed_pct: int  = 0
        self._last_command_time: float = time.time()
        self._motor_run_start: Optional[float] = None
        self._cooldown_until: Optional[float]  = None

        # --- GPIO ---
        self._mock_mode = not _LGPIO_AVAILABLE
        if self._mock_mode:
            self.get_logger().warn(
                f"lgpio not available — running in MOCK mode for '{self._actuator_name}'"
            )
            self._gpio = MockGPIO()
        else:
            self._gpio = lgpio

        self._h = self._gpio.gpiochip_open(4)
        self._setup_gpio()

        # --- ROS interfaces ---
        self._cmd_sub = self.create_subscription(
            Float64, "~/command", self._command_callback, 10,
            callback_group=self._cb_group,
        )
        self._status_pub = self.create_publisher(DiagnosticStatus, "~/status", 10)

        self._stop_srv = self.create_service(
            Trigger, "~/stop", self._stop_callback, callback_group=self._cb_group
        )
        self._extend_srv = self.create_service(
            Trigger, "~/extend", self._extend_callback, callback_group=self._cb_group
        )
        self._retract_srv = self.create_service(
            Trigger, "~/retract", self._retract_callback, callback_group=self._cb_group
        )

        self._control_timer = self.create_timer(
            0.02, self._control_loop, callback_group=self._cb_group  # 50 Hz
        )
        self._status_timer = self.create_timer(
            0.2, self._publish_status, callback_group=self._cb_group  # 5 Hz
        )

        self.get_logger().info(
            f"ActuatorDriverNode '{self._actuator_name}' ready "
            f"({'MOCK' if self._mock_mode else 'REAL GPIO'}) — "
            f"publish Float64 in [-1, 1] to ~/command"
        )

    # -----------------------------------------------------------------------
    # GPIO
    # -----------------------------------------------------------------------

    def _setup_gpio(self):
        if self._mock_mode:
            return
        lgpio.gpio_claim_output(self._h, self._rpwm_gpio, 0)
        lgpio.gpio_claim_output(self._h, self._lpwm_gpio, 0)
        lgpio.gpio_claim_output(self._h, self._en_gpio,   0)
        lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
        lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)

    # -----------------------------------------------------------------------
    # Motor helpers
    # -----------------------------------------------------------------------

    def _set_motor(self, direction: Direction, speed_pct: int):
        effective = direction
        if self._invert_direction and direction != Direction.STOPPED:
            effective = Direction.RETRACT if direction == Direction.EXTEND else Direction.EXTEND

        self._direction = direction
        self._current_speed_pct = speed_pct

        if self._mock_mode:
            return

        lgpio.gpio_write(self._h, self._en_gpio, 1)
        if effective == Direction.EXTEND:
            lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, speed_pct)
            lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)
        elif effective == Direction.RETRACT:
            lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
            lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, speed_pct)

    def _stop_motor(self):
        self._direction = Direction.STOPPED
        self._current_speed_pct = 0
        self._motor_run_start = None
        if self._mock_mode:
            return
        lgpio.tx_pwm(self._h, self._rpwm_gpio, self._pwm_frequency, 0)
        lgpio.tx_pwm(self._h, self._lpwm_gpio, self._pwm_frequency, 0)
        lgpio.gpio_write(self._h, self._en_gpio, 0)

    # -----------------------------------------------------------------------
    # Duty-cycle enforcement
    # -----------------------------------------------------------------------

    def _in_cooldown(self) -> bool:
        return bool(self._cooldown_until and time.time() < self._cooldown_until)

    def _check_duty_cycle(self) -> bool:
        """Return True if motor should be stopped due to duty-cycle limit."""
        now = time.time()
        if self._in_cooldown():
            return True
        if self._motor_run_start is None:
            self._motor_run_start = now
            return False
        if now - self._motor_run_start >= self._max_continuous_run_s:
            self.get_logger().warn(
                f"[{self._actuator_name}] Duty-cycle limit reached. "
                f"Cooling down for {self._cooldown_time_s}s."
            )
            self._cooldown_until = now + self._cooldown_time_s
            self._motor_run_start = None
            return True
        return False

    # -----------------------------------------------------------------------
    # Control loop (50 Hz)
    # -----------------------------------------------------------------------

    def _control_loop(self):
        now = time.time()

        # Watchdog
        if now - self._last_command_time > self._watchdog_timeout_s:
            if self._direction != Direction.STOPPED:
                self.get_logger().warn(
                    f"[{self._actuator_name}] Watchdog — stopping motor.",
                    throttle_duration_sec=5.0,
                )
                self._stop_motor()
            return

        # Cooldown
        if self._in_cooldown():
            if self._direction != Direction.STOPPED:
                self._stop_motor()
            return

        drive = self._drive_value

        if abs(drive) < 0.01:
            if self._direction != Direction.STOPPED:
                self._stop_motor()
            return

        direction = Direction.EXTEND if drive > 0 else Direction.RETRACT
        speed_pct = min(int(abs(drive) * self._max_speed_pct), self._max_speed_pct)

        if self._check_duty_cycle():
            self._stop_motor()
            return

        self._set_motor(direction, speed_pct)

    # -----------------------------------------------------------------------
    # ROS callbacks
    # -----------------------------------------------------------------------

    def _command_callback(self, msg: Float64):
        val = float(msg.data)
        self._drive_value = max(-1.0, min(1.0, val))
        self._last_command_time = time.time()

    def _stop_callback(self, request, response):
        self._drive_value = 0.0
        self._stop_motor()
        response.success = True
        response.message = f"[{self._actuator_name}] Stopped."
        return response

    def _extend_callback(self, request, response):
        """Blocking service: extend actuator for max_continuous_run_s then stop.

        Safe to block here because the node uses MultiThreadedExecutor +
        ReentrantCallbackGroup — other callbacks continue on separate threads.
        """
        if self._in_cooldown():
            response.success = False
            response.message = f"[{self._actuator_name}] In cooldown — cannot extend."
            return response

        self.get_logger().info(f"[{self._actuator_name}] Extend service: driving for up to {self._max_continuous_run_s}s")
        self._drive_value = 1.0
        self._last_command_time = time.time()

        # Keepalive loop: ping _last_command_time every 0.5 s so the 2 s watchdog
        # never fires while the service is intentionally running the motor.
        deadline = time.time() + self._max_continuous_run_s
        while time.time() < deadline:
            self._last_command_time = time.time()   # prevent watchdog cutoff
            if self._in_cooldown():
                break
            time.sleep(0.05)

        self._drive_value = 0.0
        self._stop_motor()
        response.success = True
        response.message = f"[{self._actuator_name}] Extend complete."
        self.get_logger().info(f"[{self._actuator_name}] Extend service: done.")
        return response

    def _retract_callback(self, request, response):
        """Blocking service: retract actuator for max_continuous_run_s then stop."""
        if self._in_cooldown():
            response.success = False
            response.message = f"[{self._actuator_name}] In cooldown — cannot retract."
            return response

        self.get_logger().info(f"[{self._actuator_name}] Retract service: driving for up to {self._max_continuous_run_s}s")
        self._drive_value = -1.0
        self._last_command_time = time.time()

        deadline = time.time() + self._max_continuous_run_s
        while time.time() < deadline:
            self._last_command_time = time.time()   # prevent watchdog cutoff
            if self._in_cooldown():
                break
            time.sleep(0.05)

        self._drive_value = 0.0
        self._stop_motor()
        response.success = True
        response.message = f"[{self._actuator_name}] Retract complete."
        self.get_logger().info(f"[{self._actuator_name}] Retract service: done.")
        return response

    # -----------------------------------------------------------------------
    # Status
    # -----------------------------------------------------------------------

    def _publish_status(self):
        uptime = time.time() - self._start_time
        in_cooldown = self._in_cooldown()

        if in_cooldown:
            level, message = DiagnosticStatus.WARN, "Cooling down"
        elif self._direction != Direction.STOPPED:
            level, message = DiagnosticStatus.OK, f"Moving ({self._direction.value})"
        else:
            level, message = DiagnosticStatus.OK, "Stopped"

        status = DiagnosticStatus()
        status.name = f"bucket_{self._actuator_name}_driver"
        status.level = level
        status.message = message
        status.values = [
            KeyValue(key="drive_value",  value=f"{self._drive_value:.2f}"),
            KeyValue(key="direction",    value=self._direction.value),
            KeyValue(key="speed_pct",    value=str(self._current_speed_pct)),
            KeyValue(key="in_cooldown",  value=str(in_cooldown).lower()),
            KeyValue(key="uptime_s",     value=f"{uptime:.1f}"),
        ]
        self._status_pub.publish(status)

    # -----------------------------------------------------------------------
    # Shutdown
    # -----------------------------------------------------------------------

    def destroy_node(self):
        self._stop_motor()
        if not self._mock_mode:
            try:
                lgpio.gpiochip_close(self._h)
            except Exception:
                pass
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
