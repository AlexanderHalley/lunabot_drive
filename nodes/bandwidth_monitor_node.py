#!/usr/bin/env python3
"""
Bandwidth Monitor Node — WiFi interface throughput monitor.

Reads /proc/net/dev at 2 Hz to compute bytes-per-second delta on wlan0
(falls back to first wl* interface found).

Publishes:
  /bandwidth_mbps (std_msgs/Float32) — WiFi throughput in Mbps
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

WARN_MBPS  = 3.0
ERROR_MBPS = 4.0


class BandwidthMonitorNode(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor_node')

        self._pub = self.create_publisher(Float32, '/bandwidth_mbps', 10)

        self._last_bytes = None
        self._last_time  = None
        self._iface      = None  # resolved on first read

        # 2 Hz
        self.create_timer(0.5, self._poll)

        self.get_logger().info('Bandwidth monitor node started (reading /proc/net/dev)')

    # ------------------------------------------------------------------
    def _find_interface(self, lines: list[str]) -> str | None:
        """Return wlan0 if present, else first wl* interface, else None."""
        candidates = []
        for line in lines:
            stripped = line.strip()
            if ':' not in stripped:
                continue
            iface = stripped.split(':')[0].strip()
            if iface == 'wlan0':
                return 'wlan0'
            if iface.startswith('wl'):
                candidates.append(iface)
        return candidates[0] if candidates else None

    def _read_rx_bytes(self) -> int | None:
        """Return total RX bytes for the chosen interface, or None on error."""
        try:
            with open('/proc/net/dev', 'r') as f:
                lines = f.readlines()
        except OSError as exc:
            self.get_logger().warn(f'Cannot read /proc/net/dev: {exc}')
            return None

        if self._iface is None:
            self._iface = self._find_interface(lines)
            if self._iface is None:
                self.get_logger().warn('No wireless interface found in /proc/net/dev')
                return None
            self.get_logger().info(f'Using interface: {self._iface}')

        for line in lines:
            stripped = line.strip()
            if stripped.startswith(self._iface + ':'):
                # Fields after the colon: rx_bytes rx_packets rx_errs ...
                data = stripped.split(':')[1].split()
                return int(data[0])  # rx_bytes

        return None

    def _poll(self):
        now   = time.monotonic()
        rx    = self._read_rx_bytes()

        if rx is None:
            return

        if self._last_bytes is None:
            self._last_bytes = rx
            self._last_time  = now
            return

        elapsed = now - self._last_time
        if elapsed < 1e-6:
            return

        delta_bytes = rx - self._last_bytes
        # Guard against counter reset / wrap
        if delta_bytes < 0:
            delta_bytes = 0

        mbps = (delta_bytes * 8) / elapsed / 1_000_000

        self._last_bytes = rx
        self._last_time  = now

        msg = Float32()
        msg.data = float(mbps)
        self._pub.publish(msg)

        if mbps > ERROR_MBPS:
            self.get_logger().error(
                f'Bandwidth CRITICAL: {mbps:.2f} Mbps (limit 4.0 Mbps)'
            )
        elif mbps > WARN_MBPS:
            self.get_logger().warn(
                f'Bandwidth WARNING: {mbps:.2f} Mbps (warn threshold 3.0 Mbps)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BandwidthMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
