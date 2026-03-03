#!/usr/bin/env python3
"""
Bucket teleop node for Lunabot rover.

Maps Nintendo Switch Pro Controller joystick inputs to bucket lift/tilt commands.
Run on the MCC laptop alongside teleop_twist_joy.

Default control mapping (Switch Pro Controller via joy_node):
  D-pad UP   → Lift +10 mm
  D-pad DOWN → Lift -10 mm
  D-pad RIGHT → Tilt +10 mm
  D-pad LEFT  → Tilt -10 mm
  Y button (3) → Home both actuators
  X button (2) → Scoop preset (lift=0, tilt=stroke_max)
  B button (1) → Dump preset (lift=stroke_max, tilt=0)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class BucketTeleopNode(Node):

    def __init__(self):
        super().__init__("bucket_teleop")

        # --- Parameters ---
        self.declare_parameter("lift_stroke_mm", 203.2)
        self.declare_parameter("tilt_stroke_mm", 254.0)
        self.declare_parameter("step_mm", 10.0)
        self.declare_parameter("dpad_axis_vertical", 7)
        self.declare_parameter("dpad_axis_horizontal", 6)
        self.declare_parameter("home_button", 3)
        self.declare_parameter("scoop_button", 2)
        self.declare_parameter("dump_button", 1)

        self._lift_stroke = self.get_parameter("lift_stroke_mm").value
        self._tilt_stroke = self.get_parameter("tilt_stroke_mm").value
        self._step_mm = self.get_parameter("step_mm").value
        self._dpad_vertical = self.get_parameter("dpad_axis_vertical").value
        self._dpad_horizontal = self.get_parameter("dpad_axis_horizontal").value
        self._home_btn = self.get_parameter("home_button").value
        self._scoop_btn = self.get_parameter("scoop_button").value
        self._dump_btn = self.get_parameter("dump_button").value

        # --- State ---
        self._lift_target: float = 0.0
        self._tilt_target: float = 0.0
        self._prev_buttons: list = []
        self._prev_dpad_v: float = 0.0
        self._prev_dpad_h: float = 0.0

        # --- Subscriptions ---
        self._joy_sub = self.create_subscription(
            Joy, "/joy", self._joy_callback, 10
        )

        # --- Publishers ---
        self._lift_pub = self.create_publisher(Float64, "/bucket/lift/command", 10)
        self._tilt_pub = self.create_publisher(Float64, "/bucket/tilt/command", 10)

        # --- Service clients ---
        self._lift_home_client = self.create_client(Trigger, "/bucket/lift/home")
        self._tilt_home_client = self.create_client(Trigger, "/bucket/tilt/home")

        self.get_logger().info("BucketTeleopNode started. Waiting for joystick input.")

    def _joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        axes = list(msg.axes)

        # --- D-pad axis reading (values: +1 = up/right, -1 = down/left) ---
        dpad_v = axes[self._dpad_vertical] if self._dpad_vertical < len(axes) else 0.0
        dpad_h = axes[self._dpad_horizontal] if self._dpad_horizontal < len(axes) else 0.0

        # Detect rising edge on d-pad (new press)
        dpad_v_pressed = dpad_v != 0.0 and self._prev_dpad_v == 0.0
        dpad_h_pressed = dpad_h != 0.0 and self._prev_dpad_h == 0.0

        if dpad_v_pressed:
            self._lift_target += self._step_mm * dpad_v
            self._lift_target = max(0.0, min(self._lift_target, self._lift_stroke))
            self._publish_lift()

        if dpad_h_pressed:
            self._tilt_target += self._step_mm * dpad_h
            self._tilt_target = max(0.0, min(self._tilt_target, self._tilt_stroke))
            self._publish_tilt()

        # --- Button handling (detect rising edge) ---
        if len(self._prev_buttons) == 0:
            self._prev_buttons = [0] * len(buttons)

        def pressed(idx):
            return (
                idx < len(buttons)
                and buttons[idx] == 1
                and (idx >= len(self._prev_buttons) or self._prev_buttons[idx] == 0)
            )

        if pressed(self._home_btn):
            self.get_logger().info("Home requested for both actuators.")
            self._call_home(self._lift_home_client, "lift")
            self._call_home(self._tilt_home_client, "tilt")
            self._lift_target = 0.0
            self._tilt_target = 0.0

        if pressed(self._scoop_btn):
            self.get_logger().info("Scoop preset: lift=0, tilt=max")
            self._lift_target = 0.0
            self._tilt_target = self._tilt_stroke
            self._publish_lift()
            self._publish_tilt()

        if pressed(self._dump_btn):
            self.get_logger().info("Dump preset: lift=max, tilt=0")
            self._lift_target = self._lift_stroke
            self._tilt_target = 0.0
            self._publish_lift()
            self._publish_tilt()

        self._prev_buttons = buttons
        self._prev_dpad_v = dpad_v
        self._prev_dpad_h = dpad_h

    def _publish_lift(self):
        msg = Float64()
        msg.data = self._lift_target
        self._lift_pub.publish(msg)
        self.get_logger().debug(f"Lift target: {self._lift_target:.1f} mm")

    def _publish_tilt(self):
        msg = Float64()
        msg.data = self._tilt_target
        self._tilt_pub.publish(msg)
        self.get_logger().debug(f"Tilt target: {self._tilt_target:.1f} mm")

    def _call_home(self, client, name: str):
        if not client.service_is_ready():
            self.get_logger().warn(f"Home service for '{name}' not ready — skipping.")
            return
        req = Trigger.Request()
        future = client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"Home '{name}': {f.result().message if f.result() else 'failed'}"
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = BucketTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
