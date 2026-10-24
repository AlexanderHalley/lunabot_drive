#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Run the competition mission: dig, dump, repeat, go home.

    ros2 run lunabot_navigation mission.py --ros-args \
        --params-file $(ros2 pkg prefix lunabot_navigation)/share/lunabot_navigation/config/mission_ucf.yaml

Waits for the start survey (/navigation/survey_done), then for each cycle
drives to the dig pose, digs, drives to the dump pose, dumps; then drives
home. Every phase change is latched on /mission/state as one line of text,
for operators, logs and the demo recorder.

Digging and dumping are PLACEHOLDERS -- a timed stop. There is no excavation
package yet; when there is, `_work()` calls its actions instead.

A failed leg is retried `retries` times (Nav2 replans from wherever the rover
ended up); a leg that still fails ends the mission as FAILED rather than
skipping ahead, because the next leg starts from the wrong place.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STATE_TOPIC = '/mission/state'
SURVEY_TOPIC = '/navigation/survey_done'


@dataclass(frozen=True)
class Step:
    """One thing the mission does: drive to a pose, or work for some seconds."""

    state: str
    pose: tuple[float, float, float] | None = None
    seconds: float = 0.0


def plan(cycles, dig_pose, dump_pose, home_pose, dig_seconds, dump_seconds) -> list[Step]:
    """The mission as a flat list of steps. Pure: tested without ROS."""
    steps = []
    for i in range(1, int(cycles) + 1):
        tag = f' ({i}/{cycles})' if cycles > 1 else ''
        steps += [
            Step('DRIVING TO EXCAVATION ZONE' + tag, pose=tuple(dig_pose)),
            Step('EXCAVATING' + tag, seconds=dig_seconds),
            Step('DRIVING TO CONSTRUCTION ZONE' + tag, pose=tuple(dump_pose)),
            Step('DEPOSITING' + tag, seconds=dump_seconds),
        ]
    steps.append(Step('RETURNING TO START', pose=tuple(home_pose)))
    return steps


def main() -> None:
    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, String

    rclpy.init()
    node = Node('mission')
    p = {
        name: node.declare_parameter(name, default).value
        for name, default in (
            ('cycles', 1),
            ('leg_timeout', 240.0),
            ('retries', 1),
            ('dig_pose', [4.0, 0.8, 0.0]),
            ('dump_pose', [6.09, 2.57, 0.0]),
            ('home_pose', [0.0, 0.0, math.pi]),
            ('dig_seconds', 8.0),
            ('dump_seconds', 6.0),
            ('wait_for_survey', True),
        )
    }
    log = node.get_logger()
    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    state_pub = node.create_publisher(String, STATE_TOPIC, latched)

    def say(state: str) -> None:
        log.info(f'mission: {state}')
        state_pub.publish(String(data=state))

    def sleep(seconds: float) -> None:
        # Sim time: a stop of N seconds is N seconds on /clock.
        end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def drive(pose) -> bool:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(pose[0]), float(pose[1])
        goal.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        sent = nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if handle is None or not handle.accepted:
            log.error('goal rejected')
            return False
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result, timeout_sec=p['leg_timeout'])
        if not result.done():
            log.error('leg timed out')
            handle.cancel_goal_async()
            return False
        return result.result().status == 4  # GoalStatus.STATUS_SUCCEEDED

    try:
        say('WAITING FOR NAVIGATION')
        nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        nav.wait_for_server()
        if p['wait_for_survey']:
            survey = {'done': None}
            node.create_subscription(
                Bool, SURVEY_TOPIC, lambda m: survey.__setitem__('done', m.data), latched
            )
            say('SURVEYING START AREA')
            while rclpy.ok() and survey['done'] is None:
                rclpy.spin_once(node, timeout_sec=0.2)
            if not survey['done']:
                log.warning('the start survey failed; carrying on without it')

        steps = plan(
            p['cycles'],
            p['dig_pose'],
            p['dump_pose'],
            p['home_pose'],
            p['dig_seconds'],
            p['dump_seconds'],
        )
        start = node.get_clock().now()
        for step in steps:
            say(step.state)
            if step.pose is None:
                sleep(step.seconds)
                continue
            for attempt in range(int(p['retries']) + 1):
                if drive(step.pose):
                    break
                if attempt < p['retries']:
                    say(f'{step.state} - RETRYING')
            else:
                say(f'FAILED: {step.state}')
                rclpy.spin(node)  # keep the latched state up
                return
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        say(f'MISSION COMPLETE in {elapsed:.0f} s')
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
