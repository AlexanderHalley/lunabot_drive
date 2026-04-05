#!/usr/bin/env python3
"""
WiFi Monitor Node — RSSI monitor.

Every 1 second, reads /proc/net/wireless for wlan0 RSSI (falls back to
first wl* interface). If /proc/net/wireless is unavailable, runs iwconfig.

Publishes:
  /wifi_rssi (std_msgs/Int32) — signal level in dBm
"""

import re
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class WifiMonitorNode(Node):
    def __init__(self):
        super().__init__('wifi_monitor_node')

        self._pub = self.create_publisher(Int32, '/wifi_rssi', 10)
        self._iface: str | None = None

        self.create_timer(1.0, self._poll)

        self.get_logger().info('WiFi monitor node started')

    # ------------------------------------------------------------------
    def _find_interface(self, lines: list[str]) -> str | None:
        """Return wlan0 or first wl* interface from /proc/net/wireless lines."""
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

    def _read_proc_wireless(self) -> int | None:
        """Read RSSI from /proc/net/wireless. Returns dBm int or None."""
        try:
            with open('/proc/net/wireless', 'r') as f:
                lines = f.readlines()
        except OSError:
            return None

        if self._iface is None:
            self._iface = self._find_interface(lines)
            if self._iface is None:
                return None
            self.get_logger().info(f'Using WiFi interface: {self._iface}')

        for line in lines:
            stripped = line.strip()
            if stripped.startswith(self._iface + ':'):
                # Format: iface: status link level noise ...
                # 'level' field is index 2 (0-based after split on ':')
                data = stripped.split(':')[1].split()
                # data[0]=link, data[1]=level, data[2]=noise
                try:
                    level = float(data[1].rstrip('.'))
                    # /proc/net/wireless reports dBm as a negative value
                    # but sometimes as an unsigned byte offset from -256
                    if level > 0:
                        level = level - 256
                    return int(level)
                except (IndexError, ValueError):
                    return None
        return None

    def _read_iwconfig(self) -> int | None:
        """Fallback: parse `iwconfig` output for Signal level."""
        try:
            result = subprocess.run(
                ['iwconfig'],
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return None

        # Look for "Signal level=-XX dBm"
        match = re.search(r'Signal level[=:](-?\d+)\s*dBm', result.stdout)
        if match:
            return int(match.group(1))
        # Some drivers omit "dBm" and report a raw value
        match = re.search(r'Signal level[=:](-?\d+)', result.stdout)
        if match:
            val = int(match.group(1))
            if val > 0:
                val = val - 256
            return val
        return None

    def _poll(self):
        rssi = self._read_proc_wireless()
        if rssi is None:
            rssi = self._read_iwconfig()
        if rssi is None:
            self.get_logger().debug('Could not read WiFi RSSI')
            return

        msg = Int32()
        msg.data = rssi
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WifiMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
