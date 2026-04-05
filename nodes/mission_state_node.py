#!/usr/bin/env python3
"""
Mission State Node — Autonomy state machine for Lunabotics competition.

Publishes:
  /autonomy_state  (std_msgs/String) at 5 Hz — TELEOP, READY, AUTONOMOUS, COMPLETE, FAILED, ESTOP
  /autonomy_cycle  (std_msgs/Int32) — current cycle count (0-based)

Subscribes:
  /autonomy_command (std_msgs/String) — arm, start_excavation, start_deposition, abort, acknowledge
  /emergency_stop   (std_msgs/Bool)   — triggers ESTOP
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


TELEOP     = 'TELEOP'
READY      = 'READY'
AUTONOMOUS = 'AUTONOMOUS'
COMPLETE   = 'COMPLETE'
FAILED     = 'FAILED'
ESTOP      = 'ESTOP'


class MissionStateNode(Node):
    def __init__(self):
        super().__init__('mission_state_node')

        self._state = TELEOP
        self._mode = None          # 'excavation' or 'deposition'
        self._cycle = 0
        self._failed_time = None   # monotonic time when FAILED was entered

        # Publishers
        self._state_pub = self.create_publisher(String, '/autonomy_state', 10)
        self._cycle_pub = self.create_publisher(Int32, '/autonomy_cycle', 10)

        # Subscribers
        self.create_subscription(String, '/autonomy_command', self._on_command, 10)
        self.create_subscription(Bool, '/emergency_stop', self._on_estop, 10)

        # Publish at 5 Hz; also handle FAILED auto-transition
        self.create_timer(0.2, self._publish_state)

        self.get_logger().info('Mission state node started — initial state: TELEOP')

    # ------------------------------------------------------------------
    # Transition helper
    # ------------------------------------------------------------------
    def _transition(self, new_state: str, reason: str = ''):
        old = self._state
        self._state = new_state
        self._failed_time = time.monotonic() if new_state == FAILED else None
        msg = f'State transition: {old} -> {new_state}'
        if reason:
            msg += f' ({reason})'
        self.get_logger().info(msg)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_command(self, msg: String):
        cmd = msg.data.strip().lower()

        if self._state == TELEOP and cmd == 'arm':
            self._transition(READY, 'arm command received')

        elif self._state == READY and cmd == 'start_excavation':
            self._mode = 'excavation'
            self._transition(AUTONOMOUS, 'excavation start confirmed')

        elif self._state == READY and cmd == 'start_deposition':
            self._mode = 'deposition'
            self._transition(AUTONOMOUS, 'deposition start confirmed')

        elif self._state == READY and cmd == 'abort':
            self._transition(TELEOP, 'abort from READY')

        elif self._state == AUTONOMOUS and cmd == 'abort':
            self._transition(FAILED, 'abort during AUTONOMOUS')

        elif self._state == COMPLETE and cmd == 'acknowledge':
            self._cycle += 1
            self._transition(TELEOP, f'cycle {self._cycle} acknowledged')

        elif self._state == ESTOP and cmd == 'acknowledge':
            self._transition(TELEOP, 'E-STOP acknowledged')

        else:
            self.get_logger().debug(
                f'Command "{cmd}" ignored in state {self._state}'
            )

    def _on_estop(self, msg: Bool):
        if msg.data and self._state != ESTOP:
            self._transition(ESTOP, 'emergency stop triggered')

    # ------------------------------------------------------------------
    # Timer: publish state + handle FAILED auto-transition
    # ------------------------------------------------------------------
    def _publish_state(self):
        # Auto-transition from FAILED to TELEOP after 5 seconds
        if self._state == FAILED and self._failed_time is not None:
            if time.monotonic() - self._failed_time >= 5.0:
                self._transition(TELEOP, 'auto-recovery after 5 s in FAILED')

        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

        cycle_msg = Int32()
        cycle_msg.data = self._cycle
        self._cycle_pub.publish(cycle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
