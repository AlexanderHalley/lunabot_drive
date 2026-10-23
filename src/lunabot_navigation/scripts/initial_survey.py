#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Turn a full circle on the spot at the start, so SLAM has seen everything.

The only camera faces forward. Without this, SLAM's memory of the start area
holds only the view down the arena, and a rover coming home -- facing the
other way -- never sees anything it recognises: no loop closure, so the drift
of the whole run stays in the pose (Isaac: 0.6-0.7 m on the return leg, and
the rover stopped 9 cm from the wall it thought was 29 cm away). One slow
turn in the start zone gives rtabmap views of the start area from every
heading, which the return leg can close a loop against.

RESULT IN ISAAC (2026-10-23): on its own it does NOT close the loop. Every
rtabmap loop-closure attempt on the return found 45-74 feature matches and
ZERO geometric inliers -- the regolith texture is random, features in one
patch look like features in many others, and visual matching looking mostly
at the floor cannot verify a single one. The survey is still the right
precondition for a loop-closure method that works on this ground (geometry:
ICP on the depth cloud against walls and boulders; or the start rail's
fiducials), so it is kept, off by default (launch arg survey:=true).

Once Nav2 is active this asks Nav2's own Spin behaviour (which collision-
checks against the costmap) for `turn` radians, then latches True on
/navigation/survey_done -- False if the spin failed. Missions should wait for
it before sending goals.
"""

from __future__ import annotations

import math

DONE_TOPIC = '/navigation/survey_done'


def main() -> None:
    import rclpy
    from nav2_msgs.action import Spin
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    rclpy.init()
    node = Node('initial_survey')
    turn = node.declare_parameter('turn', 2.0 * math.pi).value
    allowance = node.declare_parameter('time_allowance', 60.0).value
    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    done = node.create_publisher(Bool, DONE_TOPIC, latched)
    log = node.get_logger()

    def finish(ok: bool) -> None:
        done.publish(Bool(data=ok))
        # Keep the latched message available for late subscribers.
        rclpy.spin(node)

    try:
        # Nav2 up: the lifecycle manager says so.
        active = node.create_client(Trigger, '/lifecycle_manager_navigation/is_active')
        while rclpy.ok():
            if active.wait_for_service(timeout_sec=2.0):
                future = active.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                if future.result() is not None and future.result().success:
                    break
            log.info('waiting for Nav2 to become active', throttle_duration_sec=10.0)
            rclpy.spin_once(node, timeout_sec=1.0)

        spin = ActionClient(node, Spin, 'spin')
        spin.wait_for_server()
        goal = Spin.Goal()
        goal.target_yaw = float(turn)
        goal.time_allowance.sec = int(allowance)
        log.info(f'surveying: turning {math.degrees(turn):.0f} deg on the spot')
        sent = spin.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            log.error('survey spin was rejected')
            finish(False)
            return
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result)
        ok = result.result().status == 4  # GoalStatus.STATUS_SUCCEEDED
        if ok:
            log.info('survey done')
        else:
            log.error(f'survey spin failed (error {result.result().result.error_code})')
        finish(ok)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
