#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Never command a tighter turn than the rover can drive.

    controller_server -> cmd_vel_nav_unlimited -> THIS -> cmd_vel_nav_unsmoothed
                                                           -> velocity_smoother

The rover cannot turn on the spot in sand, and below a commanded radius of
about 1 m it does not turn at all: the side forces lock, the rover stops, and
the wheels spin in place (lunabot_sim/robot/soil.py; measured in Isaac). The
wheel odometry then counts metres the rover never drove, and every frame
built on odom drifts away.

Regulated Pure Pursuit has no curvature limit. It slows down in bends tighter
than its regulated_linear_scaling_min_radius, but keeps turning at the
geometric curvature to its carrot -- which, while the rover lags a Hybrid-A*
path, is often far tighter than 1 m. So this clamps the turn rate to
|v| / min_turning_radius, and to zero when the rover is not moving forward or
back. Linear speed passes through untouched: slowing down is the follower's
business, and the smoother's.

Behaviour-server recoveries do not come through here. They only drive
straight (BackUp) or stand still (Wait).
"""

from __future__ import annotations

V_EPS = 1e-3
"""m/s. Below this the rover is not moving, so it may not turn either."""


def limit(linear: float, angular: float, min_turning_radius: float) -> tuple[float, float]:
    """Return (linear, angular) with the turn no tighter than min_turning_radius."""
    if min_turning_radius <= 0.0:
        return linear, angular
    if abs(linear) < V_EPS:
        return linear, 0.0
    cap = abs(linear) / min_turning_radius
    return linear, max(-cap, min(cap, angular))


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from rclpy.node import Node

    class CurvatureLimiter(Node):
        def __init__(self):
            super().__init__('curvature_limiter')
            self._radius = self.declare_parameter('min_turning_radius', 1.0).value
            self._pub = self.create_publisher(TwistStamped, 'cmd_vel_out', 10)
            self.create_subscription(TwistStamped, 'cmd_vel_in', self._on_cmd, 10)
            self._clamped = 0
            self._seen = 0
            self.create_timer(10.0, self._report)
            self.get_logger().info(f'turns limited to radius >= {self._radius:.2f} m')

        def _on_cmd(self, msg: TwistStamped) -> None:
            linear, angular = limit(msg.twist.linear.x, msg.twist.angular.z, self._radius)
            self._seen += 1
            if angular != msg.twist.angular.z:
                self._clamped += 1
            msg.twist.angular.z = angular
            msg.twist.linear.x = linear
            self._pub.publish(msg)

        def _report(self) -> None:
            if self._clamped:
                self.get_logger().info(
                    f'clamped {self._clamped} of {self._seen} commands in the last 10 s'
                )
            self._clamped = self._seen = 0

    rclpy.init()
    node = CurvatureLimiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
