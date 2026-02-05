#!/usr/bin/env python3
"""
Bandwidth Monitor Node

Monitors ROS2 topic bandwidth for Lunabotics competition compliance.
The competition has a 4 Mbps average bandwidth limit.
Scoring: TAB * -30 + 120 (max 120 pts). Lower bandwidth = more points.

Usage:
    ros2 run lunabot_drive bandwidth_monitor.py
    ros2 run lunabot_drive bandwidth_monitor.py --ros-args -p report_interval:=5.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time


class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor')

        self.declare_parameter('report_interval', 5.0)  # seconds between reports
        self.declare_parameter('warn_threshold_mbps', 3.0)  # warn above this
        self.declare_parameter('critical_threshold_mbps', 4.0)  # critical above this

        self.report_interval = self.get_parameter('report_interval').value
        self.warn_threshold = self.get_parameter('warn_threshold_mbps').value
        self.critical_threshold = self.get_parameter('critical_threshold_mbps').value

        # Track bytes per topic
        self.topic_bytes = {}
        self.topic_counts = {}
        self.last_report_time = time.monotonic()

        # Use best-effort QoS to match typical sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribe to known heavy topics
        self.create_subscription(
            PointCloud2, '/oak/points',
            lambda msg: self._track('/oak/points', msg), sensor_qos)
        self.create_subscription(
            Imu, '/oak/imu/data',
            lambda msg: self._track('/oak/imu/data', msg), sensor_qos)
        self.create_subscription(
            Odometry, '/odom',
            lambda msg: self._track('/odom', msg), reliable_qos)
        self.create_subscription(
            JointState, '/joint_states',
            lambda msg: self._track('/joint_states', msg), reliable_qos)
        self.create_subscription(
            Twist, '/cmd_vel',
            lambda msg: self._track('/cmd_vel', msg), reliable_qos)

        # Report timer
        self.create_timer(self.report_interval, self._report)

        self.get_logger().info(
            f'Bandwidth monitor started (report every {self.report_interval}s, '
            f'warn at {self.warn_threshold} Mbps, critical at {self.critical_threshold} Mbps)'
        )

    def _estimate_size(self, topic, msg):
        """Estimate message size in bytes."""
        if isinstance(msg, PointCloud2):
            return msg.row_step * msg.height + 64  # data + header overhead
        elif isinstance(msg, Imu):
            return 296  # fixed-size message
        elif isinstance(msg, Odometry):
            return 720  # fixed-size with covariance
        elif isinstance(msg, JointState):
            n = len(msg.name)
            return 64 + n * 40  # header + per-joint data
        elif isinstance(msg, Twist):
            return 48  # 6 doubles
        return 256  # fallback estimate

    def _track(self, topic, msg):
        """Track bytes received on a topic."""
        size = self._estimate_size(topic, msg)
        self.topic_bytes[topic] = self.topic_bytes.get(topic, 0) + size
        self.topic_counts[topic] = self.topic_counts.get(topic, 0) + 1

    def _report(self):
        """Log bandwidth report."""
        now = time.monotonic()
        elapsed = now - self.last_report_time
        if elapsed < 0.1:
            return

        total_bytes = 0
        lines = []

        for topic in sorted(self.topic_bytes.keys()):
            bytes_count = self.topic_bytes[topic]
            msg_count = self.topic_counts[topic]
            rate_bps = (bytes_count * 8) / elapsed
            rate_mbps = rate_bps / 1_000_000
            hz = msg_count / elapsed
            total_bytes += bytes_count
            lines.append(f'  {topic}: {rate_mbps:.3f} Mbps ({hz:.1f} Hz)')

        total_bps = (total_bytes * 8) / elapsed
        total_mbps = total_bps / 1_000_000

        # Compute competition score impact
        score = max(0, min(120, total_mbps * -30 + 120))

        report = f'\n--- Bandwidth Report ({elapsed:.1f}s window) ---\n'
        report += '\n'.join(lines) if lines else '  (no topics received)'
        report += f'\n  TOTAL: {total_mbps:.3f} Mbps'
        report += f'\n  Competition bandwidth score: {score:.0f}/120 pts'
        report += '\n---'

        if total_mbps >= self.critical_threshold:
            self.get_logger().error(report)
        elif total_mbps >= self.warn_threshold:
            self.get_logger().warn(report)
        else:
            self.get_logger().info(report)

        # Reset counters
        self.topic_bytes.clear()
        self.topic_counts.clear()
        self.last_report_time = now


def main(args=None):
    rclpy.init(args=args)
    node = BandwidthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
