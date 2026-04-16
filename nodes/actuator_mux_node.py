#!/usr/bin/env python3
"""
Actuator command multiplexer for Lunabot.

Priority/timeout-based Float64 topic mux — mirrors the cmd_vel_mux pattern
but for actuator command topics (Float64 in [-1.0, 1.0]).

The highest-priority active input whose last message arrived within its
timeout window wins; its value is forwarded to the output topic at 50 Hz.
If no input is active, nothing is published — the actuator driver's 2-second
watchdog then stops the motor safely.

Source switches are logged so the operator can see who is in control.

Parameters
----------
  output_topic      : str         — absolute topic to publish the selected command
  subscriber_names  : string[]    — ordered list of input names to configure
                                    (e.g. ["teleop", "autonomy"])
  subscribers.<name>.topic      : str   — input topic
  subscribers.<name>.priority   : int   — higher number wins (teleop=10, autonomy=1)
  subscribers.<name>.timeout    : float — seconds; input is "active" if last message
                                          arrived within this window
  subscribers.<name>.short_desc : str   — label used in log messages only

Topic routing (default config from actuator_mux_lunabot.yaml):
  Teleop     → /bucket/{lift,tilt}_mux/input/teleop   (priority 10)
  Autonomy   → /bucket/{lift,tilt}_mux/input/autonomy (priority  1)
  Mux output → /bucket/{lift,tilt}/lift_driver/command
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class ActuatorMuxNode(Node):

    def __init__(self):
        super().__init__('actuator_mux')

        self.declare_parameter('output_topic', 'command')
        self.declare_parameter('subscriber_names', ['teleop', 'autonomy'])

        output_topic = self.get_parameter('output_topic').value
        names = (
            self.get_parameter('subscriber_names')
            .get_parameter_value()
            .string_array_value
        )

        self._inputs = []  # list of dicts, sorted priority-desc after init

        for name in names:
            self.declare_parameter(f'subscribers.{name}.topic',      '')
            self.declare_parameter(f'subscribers.{name}.priority',   0)
            self.declare_parameter(f'subscribers.{name}.timeout',    0.5)
            self.declare_parameter(f'subscribers.{name}.short_desc', name)

            topic      = self.get_parameter(f'subscribers.{name}.topic').value
            priority   = self.get_parameter(f'subscribers.{name}.priority').value
            timeout    = self.get_parameter(f'subscribers.{name}.timeout').value
            short_desc = self.get_parameter(f'subscribers.{name}.short_desc').value

            if not topic:
                self.get_logger().warn(f'Input "{name}" has no topic configured — skipping')
                continue

            entry = {
                'name':       name,
                'priority':   priority,
                'timeout':    timeout,
                'short_desc': short_desc,
                'last_time':  -float('inf'),
                'last_value': 0.0,
            }

            # Closure so each subscriber captures its own entry dict
            def _make_cb(e):
                def _cb(msg: Float64):
                    e['last_value'] = msg.data
                    e['last_time']  = time.monotonic()
                return _cb

            self.create_subscription(Float64, topic, _make_cb(entry), 10)
            self._inputs.append(entry)
            self.get_logger().info(
                f'  input "{name}" priority={priority} timeout={timeout}s → {topic}'
            )

        # Sort once; iteration order determines priority during the mux loop
        self._inputs.sort(key=lambda x: x['priority'], reverse=True)

        self._pub = self.create_publisher(Float64, output_topic, 10)
        self._active_source: str | None = None

        self.create_timer(0.02, self._mux_loop)  # 50 Hz

        self.get_logger().info(
            f'ActuatorMuxNode ready — {len(self._inputs)} input(s) → {output_topic}'
        )

    # ── Mux loop (50 Hz) ─────────────────────────────────────────────────────

    def _mux_loop(self):
        now = time.monotonic()
        winner = None

        for inp in self._inputs:   # already priority-desc
            if now - inp['last_time'] <= inp['timeout']:
                winner = inp
                break

        if winner is None:
            if self._active_source is not None:
                self.get_logger().info(
                    'No active input — mux silent (actuator watchdog will stop motor)',
                    throttle_duration_sec=5.0,
                )
                self._active_source = None
            return

        if winner['name'] != self._active_source:
            self.get_logger().info(
                f'Source: {self._active_source} → {winner["name"]} '
                f'({winner["short_desc"]}, priority {winner["priority"]})'
            )
            self._active_source = winner['name']

        msg = Float64()
        msg.data = winner['last_value']
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
