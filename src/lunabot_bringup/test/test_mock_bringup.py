#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Bring the whole stack up on mock hardware and drive it.

The highest-value test in the workspace, and the one CI leans on. No hardware,
no GPU, no network.

If /odom moves in response to /cmd_vel, then: the xacro expanded, the URDF
parsed, robot_state_publisher accepted it, controller_manager loaded the
hardware plugin, both controllers claimed their interfaces, mock_components
integrated the dynamics, diff_drive_controller's kinematics ran, and TF is
connected. One assertion covering the entire chain.

This is also where the TwistStamped question gets settled empirically rather
than by reading release notes -- see _publisher_for_cmd_vel below.
"""

import time
import unittest

import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

WHEEL_JOINTS = {
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
}


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lunabot_bringup'), 'launch', 'robot.launch.py']
            )
        ),
        launch_arguments={
            'hw': 'mock',
            'rviz': 'false',
            # teleop off: joy_node opens /dev/input/js0, which does not exist
            # on a CI runner, and the test publishes to /cmd_vel directly
            # anyway. twist_mux comes with teleop, so with it off this test
            # talks to diff_drive_controller through the same remapping the
            # mux would use.
            'teleop': 'false',
            'camera': 'false',
        }.items(),
    )

    return LaunchDescription(
        [
            robot,
            # Controller spawning is chained through an OnProcessExit handler,
            # so the graph is not complete the moment launch returns. The
            # tests below poll with their own timeouts; this delay just keeps
            # the first one from burning its budget on startup.
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestMockBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_mock_bringup')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, what):
        """Poll until predicate() is truthy. Fails with `what` on timeout."""
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.fail(f'timed out after {timeout}s waiting for {what}')

    def collect(self, topic, msg_type, timeout, qos=10):
        """Subscribe and return every message seen within the timeout."""
        received = []
        sub = self.node.create_subscription(msg_type, topic, received.append, qos)
        try:
            self.wait_for(lambda: len(received) > 0, timeout, f'a message on {topic}')
            self.spin(1.0)
            return received
        finally:
            self.node.destroy_subscription(sub)

    def _publisher_for_cmd_vel(self):
        """Return (publisher, make_message) matching whatever /cmd_vel expects.

        In ROS 2 Jazzy diff_drive_controller moved to TwistStamped and dropped
        the use_stamped_vel parameter. Rather than hardcode an assumption that
        will be wrong on some distro, ask the graph what the subscriber wants.

        This is deliberately not a strict assertion: the point of the test is
        that the robot drives, not which message type carries the command.
        docs/TOPIC_FRAME_CONTRACT.md is where the answer gets recorded once
        you have run this.
        """
        self.wait_for(
            lambda: any(name == '/cmd_vel' for name, _ in self.node.get_topic_names_and_types()),
            30.0,
            '/cmd_vel to appear in the graph',
        )
        types = dict(self.node.get_topic_names_and_types())['/cmd_vel']

        if 'geometry_msgs/msg/TwistStamped' in types:

            def make(linear, angular):
                msg = TwistStamped()
                msg.header.frame_id = 'base_link'
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.twist.linear.x = linear
                msg.twist.angular.z = angular
                return msg

            return self.node.create_publisher(TwistStamped, '/cmd_vel', 10), make

        def make(linear, angular):
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            return msg

        return self.node.create_publisher(Twist, '/cmd_vel', 10), make

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_robot_description_is_published(self):
        """/robot_description is latched.

        robot_state_publisher latches it, so a late subscriber must still receive it -- hence
        transient local.
        """
        from std_msgs.msg import String

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        messages = self.collect('/robot_description', String, timeout=30.0, qos=qos)
        description = messages[-1].data
        self.assertIn('<robot', description)
        self.assertIn('mock_components/GenericSystem', description)

    def test_02_all_four_wheels_report_state(self):
        messages = self.collect('/joint_states', JointState, timeout=30.0)
        names = set(messages[-1].name)
        self.assertTrue(
            WHEEL_JOINTS <= names,
            f'joint_state_broadcaster is missing {sorted(WHEEL_JOINTS - names)}',
        )

    def test_03_tf_tree_reaches_the_camera(self):
        """base_link -> oak_d_rgb_camera_optical_frame must exist.

        On the 2026 robot this transform came from a static_transform_publisher
        that only ran inside one RViz launch file. Now it comes from the URDF,
        so it exists whenever the robot does.
        """
        qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        messages = self.collect('/tf_static', TFMessage, timeout=30.0, qos=qos)

        edges = {(t.header.frame_id, t.child_frame_id) for m in messages for t in m.transforms}
        children = {child for _, child in edges}

        for frame in (
            'base_footprint',
            'oak_d_link',
            'oak_d_rgb_camera_optical_frame',
            'oak_d_imu_frame',
        ):
            self.assertIn(frame, children, f'{frame} has no parent in /tf_static')

    def test_04_odometry_responds_to_command(self):
        """The one that matters.

        mock_components integrates position from the velocity command
        (calculate_dynamics: true), so diff_drive_controller sees real wheel
        motion and produces real odometry. A stationary /odom here means
        something in the chain is not connected.
        """
        odom = []
        sub = self.node.create_subscription(Odometry, '/odom', odom.append, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        self.wait_for(lambda: len(odom) > 0, 30.0, 'the first /odom message')
        start = odom[-1].pose.pose.position

        publisher, make = self._publisher_for_cmd_vel()

        # 3 seconds at 0.3 m/s. Publishing continuously matters:
        # cmd_vel_timeout is 0.5 s, so a single message would be obeyed
        # briefly and then watchdogged to a stop.
        end = time.time() + 3.0
        while time.time() < end:
            publisher.publish(make(0.3, 0.0))
            rclpy.spin_once(self.node, timeout_sec=0.05)

        final = odom[-1].pose.pose.position
        travelled = ((final.x - start.x) ** 2 + (final.y - start.y) ** 2) ** 0.5

        # Generous bounds. This asserts "the robot moved roughly the right
        # distance in roughly the right direction", not the accuracy of the
        # kinematics -- the wheel dimensions are still placeholders.
        self.assertGreater(travelled, 0.2, f'robot did not move: {travelled:.3f} m')
        self.assertLess(travelled, 3.0, f'robot moved implausibly far: {travelled:.3f} m')
        self.assertGreater(final.x, start.x, 'robot moved backwards on a forward command')

    def test_05_odom_frames_match_the_contract(self):
        messages = self.collect('/odom', Odometry, timeout=30.0)
        self.assertEqual(messages[-1].header.frame_id, 'odom')
        self.assertEqual(messages[-1].child_frame_id, 'base_link')

    def test_06_exactly_one_publisher_owns_odom_to_base_link(self):
        """Exactly one node publishes odom -> base_link.

        With odom_source:=wheel that is diff_drive_controller, and nothing else may be publishing
        it.
        """
        tf = []
        sub = self.node.create_subscription(TFMessage, '/tf', tf.append, 100)
        self.addCleanup(self.node.destroy_subscription, sub)

        self.wait_for(lambda: len(tf) > 0, 30.0, 'a dynamic transform on /tf')
        self.spin(2.0)

        edges = {(t.header.frame_id, t.child_frame_id) for m in tf for t in m.transforms}
        self.assertIn(('odom', 'base_link'), edges)

        publishers = self.node.get_publishers_info_by_topic('/tf')
        names = [p.node_name for p in publishers]
        self.assertEqual(
            len(names),
            1,
            f'expected one /tf publisher with odom_source:=wheel, got {names}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_everything_exited_cleanly(self, proc_info):
        """Every process exited zero.

        A non-zero exit from any process usually means a plugin failed to load, which the runtime
        tests above can mistake for a slow start.
        """
        launch_testing.asserts.assertExitCodes(proc_info)
