#!/usr/bin/env python3
"""
Pattern driver — runs the rover through a scripted sequence of motion
primitives using closed-loop feedback on /odom.

Primitives:
  ('straight', distance_m, linear_speed_mps)
  ('turn',     angle_rad,  angular_speed_radps)   # in-place, +ve = left/CCW
  ('arc',      distance_m, radius_m, linear_speed_mps)  # +radius = left turn
  ('goto',     x_m, y_m, linear_speed_mps)        # rotate to face, drive to point

Default pattern:
  1. forward 5 m
  2. turn right 90 deg
  3. left arc 4 m, radius 2 m
  4. drive straight back to the origin

Publishes geometry_msgs/Twist on /cmd_vel at 20 Hz. Stops automatically
when the pattern finishes. Ctrl-C cleanly publishes a zero Twist.

Usage:
    ros2 run lunabot_drive pattern_driver_node
    ros2 run lunabot_drive pattern_driver_node --ros-args -p linear_speed:=0.2
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PatternDriver(Node):
    def __init__(self):
        super().__init__('pattern_driver')

        self.declare_parameter('linear_speed', 0.25)        # m/s
        self.declare_parameter('angular_speed', 0.6)         # rad/s
        self.declare_parameter('position_tolerance', 0.08)   # m
        self.declare_parameter('heading_tolerance', 0.05)    # rad
        self.declare_parameter('control_rate', 20.0)         # Hz

        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('angular_speed').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.yaw_tol = self.get_parameter('heading_tolerance').value
        rate = self.get_parameter('control_rate').value

        # Default pattern — edit here or replace with a YAML loader later.
        self.pattern = [
            ('straight', 5.0, v),
            ('turn',    -math.pi / 2, w),    # right 90 deg
            ('arc',      4.0, 2.0, v),       # left arc, radius 2 m
            ('goto',     0.0, 0.0, v),       # return to origin
        ]

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)

        self.have_odom = False
        self.x = self.y = self.yaw = 0.0

        self.step_idx = 0
        self.step_started = False
        self.start_x = self.start_y = self.start_yaw = 0.0
        self.goto_phase = 'rotate'  # for 'goto': rotate -> drive

        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'Pattern driver ready — {len(self.pattern)} steps queued')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.have_odom = True

    def publish(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

    def stop(self):
        self.publish(0.0, 0.0)

    def finish_step(self):
        self.stop()
        self.get_logger().info(
            f'Step {self.step_idx + 1}/{len(self.pattern)} done '
            f'(pose: x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}°)')
        self.step_idx += 1
        self.step_started = False
        self.goto_phase = 'rotate'

    def tick(self):
        if not self.have_odom:
            return

        if self.step_idx >= len(self.pattern):
            self.stop()
            self.get_logger().info('Pattern complete — shutting down')
            self.timer.cancel()
            rclpy.shutdown()
            return

        step = self.pattern[self.step_idx]
        kind = step[0]

        if not self.step_started:
            self.start_x, self.start_y, self.start_yaw = self.x, self.y, self.yaw
            self.step_started = True
            self.get_logger().info(
                f'Step {self.step_idx + 1}/{len(self.pattern)}: {step}')

        if kind == 'straight':
            _, dist, v = step
            travelled = math.hypot(self.x - self.start_x, self.y - self.start_y)
            if travelled >= dist:
                self.finish_step()
            else:
                self.publish(math.copysign(v, dist), 0.0)

        elif kind == 'turn':
            _, angle, w = step
            turned = wrap(self.yaw - self.start_yaw)
            if abs(turned) >= abs(angle) - self.yaw_tol:
                self.finish_step()
            else:
                self.publish(0.0, math.copysign(w, angle))

        elif kind == 'arc':
            _, dist, radius, v = step
            travelled = math.hypot(self.x - self.start_x, self.y - self.start_y)
            if travelled >= dist:
                self.finish_step()
            else:
                # +radius = CCW (left), -radius = CW (right)
                self.publish(v, v / radius)

        elif kind == 'goto':
            _, gx, gy, v = step
            dx, dy = gx - self.x, gy - self.y
            dist = math.hypot(dx, dy)
            if dist < self.pos_tol:
                self.finish_step()
                return
            target_yaw = math.atan2(dy, dx)
            heading_err = wrap(target_yaw - self.yaw)
            if self.goto_phase == 'rotate':
                if abs(heading_err) < self.yaw_tol:
                    self.goto_phase = 'drive'
                else:
                    w = self.get_parameter('angular_speed').value
                    self.publish(0.0, math.copysign(w, heading_err))
            else:
                # Slow down inside 0.5 m and correct heading on the fly.
                lin = min(v, max(0.05, v * (dist / 0.5)))
                ang = max(-1.0, min(1.0, 1.5 * heading_err))
                self.publish(lin, ang)

        else:
            self.get_logger().error(f'Unknown primitive: {kind} — skipping')
            self.finish_step()


def main():
    rclpy.init()
    node = PatternDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
