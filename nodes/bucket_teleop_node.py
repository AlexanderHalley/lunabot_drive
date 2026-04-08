#!/usr/bin/env python3
"""
Bucket teleop node for Lunabot rover.

Maps Nintendo Switch Pro Controller d-pad to bucket lift/tilt drive commands.

Control mapping:
  D-pad UP    -> Lift extend   (+1.0)
  D-pad DOWN  -> Lift retract  (-1.0)
  D-pad RIGHT -> Tilt extend   (+1.0)
  D-pad LEFT  -> Tilt retract  (-1.0)
  Release     -> Stop          (0.0)

Axis indices
------------
On Linux with the `hid-nintendo` kernel driver (standard on Ubuntu 22.04+
and the Pi 5) the Switch Pro Controller reports its d-pad on axes 6
(horizontal) and 7 (vertical). Each axis is ternary: -1.0, 0.0, +1.0.
Override via `dpad_axis_horizontal` / `dpad_axis_vertical` params if your
distro uses a different joystick driver.

Publishes at 10 Hz while any d-pad input is active so the actuator driver
watchdog stays fed. Publishes 0.0 on release so the motor stops cleanly.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy


class BucketTeleopNode(Node):

    def __init__(self):
        super().__init__("bucket_teleop")

        self.declare_parameter("dpad_axis_vertical",   7)
        self.declare_parameter("dpad_axis_horizontal", 6)
        self.declare_parameter("publish_rate_hz",      10.0)

        self._dpad_vertical   = self.get_parameter("dpad_axis_vertical").value
        self._dpad_horizontal = self.get_parameter("dpad_axis_horizontal").value
        publish_rate          = self.get_parameter("publish_rate_hz").value

        self._lift_drive: float = 0.0
        self._tilt_drive: float = 0.0

        self._joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        self._lift_pub = self.create_publisher(
            Float64, "/bucket/lift/lift_driver/command", 10
        )
        self._tilt_pub = self.create_publisher(
            Float64, "/bucket/tilt/tilt_driver/command", 10
        )

        # Publish at a fixed rate to keep the actuator watchdog fed
        self._pub_timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            "BucketTeleopNode ready — d-pad UP/DOWN=lift, RIGHT/LEFT=tilt"
        )

    def _joy_callback(self, msg: Joy):
        axes = msg.axes

        dpad_v = axes[self._dpad_vertical]   if self._dpad_vertical   < len(axes) else 0.0
        dpad_h = axes[self._dpad_horizontal] if self._dpad_horizontal < len(axes) else 0.0

        # d-pad values are +1 / 0 / -1; pass straight through as drive signal
        self._lift_drive = float(dpad_v)
        self._tilt_drive = float(dpad_h)

    def _publish(self):
        lift_msg = Float64()
        lift_msg.data = self._lift_drive
        self._lift_pub.publish(lift_msg)

        tilt_msg = Float64()
        tilt_msg.data = self._tilt_drive
        self._tilt_pub.publish(tilt_msg)


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
