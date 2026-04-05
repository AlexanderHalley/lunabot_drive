#!/usr/bin/env python3
"""
Health Monitor Node — Node liveness checker.

Every 1 second, runs `ros2 node list` and checks for expected nodes.
A node is considered dead after 3 consecutive missed polls.

Publishes:
  /node_health (std_msgs/String) — JSON: {"node_name": true/false, ..., "timestamp": float}
"""

import json
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


EXPECTED_NODES = [
    'sparkflex_driver',
    'robot_localization',
    'apriltag_ros',
    'rplidar_node',
    'oak',
    'bt_navigator',
    'controller_server',
    'rosbridge_server',
    'bucket_controller',
]

DEAD_THRESHOLD = 3  # consecutive misses before marking dead


class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor_node')

        self._pub = self.create_publisher(String, '/node_health', 10)

        # Miss counter per node (counts consecutive polls where node was absent)
        self._miss_count: dict[str, int] = {n: 0 for n in EXPECTED_NODES}
        # Current liveness state (True = alive)
        self._alive: dict[str, bool] = {n: False for n in EXPECTED_NODES}

        self.create_timer(1.0, self._poll)

        self.get_logger().info('Health monitor node started')

    # ------------------------------------------------------------------
    def _get_running_nodes(self) -> set[str] | None:
        """Run `ros2 node list` and return a set of short node names."""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            self.get_logger().warn(f'ros2 node list failed: {exc}')
            return None

        nodes = set()
        for line in result.stdout.splitlines():
            # Lines look like "/namespace/node_name" or "/node_name"
            name = line.strip().lstrip('/')
            if '/' in name:
                # Take the last component
                name = name.rsplit('/', 1)[-1]
            if name:
                nodes.add(name)
        return nodes

    def _poll(self):
        running = self._get_running_nodes()

        for node in EXPECTED_NODES:
            if running is None:
                # Command failed; don't change miss counts
                break

            if node in running:
                self._miss_count[node] = 0
                self._alive[node] = True
            else:
                self._miss_count[node] += 1
                was_alive = self._alive[node]
                if self._miss_count[node] >= DEAD_THRESHOLD:
                    if was_alive:
                        self.get_logger().warn(
                            f'Node "{node}" has been absent for '
                            f'{self._miss_count[node]} consecutive polls — marking DEAD'
                        )
                    self._alive[node] = False

        # Publish health JSON
        payload = dict(self._alive)
        payload['timestamp'] = time.time()
        msg = String()
        msg.data = json.dumps(payload)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HealthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
