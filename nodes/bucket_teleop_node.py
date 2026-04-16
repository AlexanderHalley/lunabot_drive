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

Topic routing (via actuator mux):
  Publishes to /bucket/lift_mux/input/teleop and /bucket/tilt_mux/input/teleop.
  The actuator_mux_node forwards the highest-priority active input to the
  actuator drivers. GUI (dashboard) has priority 10 (highest); teleop has
  priority 5, overriding autonomy (1) when the d-pad is active.

  Publishing discipline: only publish while the d-pad is non-zero, plus one
  explicit 0.0 stop message on release. After that, go silent so the mux
  timeout expires and lower-priority inputs (GUI, autonomy) can win.

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
        # True while we are actively publishing a non-zero command. Used to
        # send exactly one 0.0 stop message on d-pad release, then go quiet so
        # the mux timeout expires and lower-priority inputs can win.
        self._lift_active: bool = False
        self._tilt_active: bool = False

        self._joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)

        self._lift_pub = self.create_publisher(
            Float64, "/bucket/lift_mux/input/teleop", 10
        )
        self._tilt_pub = self.create_publisher(
            Float64, "/bucket/tilt_mux/input/teleop", 10
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
        # Lift: publish while active; send one explicit stop on release; then
        # stay silent so the mux timeout expires and lower-priority inputs win.
        if self._lift_drive != 0.0:
            self._lift_active = True
            self._lift_pub.publish(Float64(data=self._lift_drive))
        elif self._lift_active:
            self._lift_active = False
            self._lift_pub.publish(Float64(data=0.0))

        # Tilt: same logic.
        if self._tilt_drive != 0.0:
            self._tilt_active = True
            self._tilt_pub.publish(Float64(data=self._tilt_drive))
        elif self._tilt_active:
            self._tilt_active = False
            self._tilt_pub.publish(Float64(data=0.0))


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
