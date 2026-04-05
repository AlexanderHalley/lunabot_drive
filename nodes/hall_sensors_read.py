#!/usr/bin/env python3
"""
Reads and logs distance travelled by the lift and tilt actuators
by subscribing to their ~/position topics (already in mm).

Note: we are not using GPIOs and we directly use actuator_driver_node

Flowchart:

Physical Hall Sensors
        ↓ (GPIO - lgpio)
actuator_driver_node.py   ← runs on Pi, handles all hardware
        ↓ (ROS2 topic, mm)
hall_sensors_read.py   ← just subscribes, no hardware needed


Topics consumed:
  /bucket/lift/actuator_driver/position  (std_msgs/Float64, mm)
  /bucket/tilt/actuator_driver/position  (std_msgs/Float64, mm)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ActuatorPositionReader(Node):
    def __init__(self):
        super().__init__("actuator_position_reader")

        self._lift_position_mm = None
        self._tilt_position_mm = None

        self.create_subscription(
            Float64,
            "/bucket/lift/actuator_driver/position",
            self._lift_callback,
            10,
        )
        self.create_subscription(
            Float64,
            "/bucket/tilt/actuator_driver/position",
            self._tilt_callback,
            10,
        )

        self.get_logger().info("Hall sensor position reader started (lift + tilt).")

    def _lift_callback(self, msg: Float64):
        self._lift_position_mm = msg.data
        self.get_logger().info(f"Lift position: {self._lift_position_mm:.2f} mm")

    def _tilt_callback(self, msg: Float64):
        self._tilt_position_mm = msg.data
        self.get_logger().info(f"Tilt position: {self._tilt_position_mm:.2f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorPositionReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
