#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Bring the whole stack up on hw:=sim, against a stand-in for Isaac.

The sibling of test_mock_bringup.py, and the reason it exists is narrow: until
now NOTHING exercised hw:=sim. The sim path differs from mock in four places
that can each fail silently --

  * a different hardware plugin (topic_based_ros2_control/TopicBasedSystem),
    which has to be built from source and whose parameters are named in the
    xacro rather than in controllers.yaml,
  * two extra topics, /isaac/joint_states and /isaac/joint_commands, whose
    names live in three files that must agree,
  * use_sim_time everywhere, where the failure mode is a stack that hangs at
    time zero with no error and no log line,
  * wrapped joint positions, where the failure mode is odometry that goes
    backwards twice per wheel revolution.

-- and every one of them would otherwise be discovered on the day the GPU
machine is first switched on, with a rover in front of you.

isaac_double.py stands in for Isaac. It is not a simulator: it is the ROS
surface of one, so this test proves the plumbing and proves nothing about
physics. See its docstring, and docs/SIM_ACCEPTANCE.md for what still has to
be checked by hand against the real thing.
"""

import subprocess
import sys
import time
import unittest
from pathlib import Path

import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

WHEEL_JOINTS = {
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
}

# The two controllers hw:=mock brings up, by the same names. "Same controllers,
# same names, either backend" is the acceptance criterion in docs/SIM_ISAAC.md.
EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
    'diff_drive_controller': 'diff_drive_controller/DiffDriveController',
}

ISAAC_DOUBLE = Path(__file__).resolve().parent / 'isaac_double.py'


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    # Started with sys.executable rather than `ros2 run`: the double is a test
    # fixture, deliberately not installed, so that nothing in the robot can
    # come to depend on it. See its docstring.
    isaac = ExecuteProcess(
        cmd=[sys.executable, str(ISAAC_DOUBLE)],
        name='isaac_double',
        output='screen',
    )

    # sim.launch.py rather than robot.launch.py, so the wait-for-clock barrier
    # is under test too. It is the guard against the worst failure mode in the
    # sim path, and an untested guard is a guess.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('lunabot_bringup'), 'launch', 'sim.launch.py'])
        ),
        launch_arguments={
            'rviz': 'false',
            # joy_node opens /dev/input/js0, which no CI runner has. The test
            # publishes to /cmd_vel directly, which is where twist_mux would
            # have put it.
            'teleop': 'false',
            # rtabmap needs camera topics, and the double publishes none: it
            # emulates the bridge graphs the CONTROL loop needs, not the
            # renderer. SLAM in sim is a manual check.
            'slam': 'none',
            'perception': 'false',
        }.items(),
    )

    return LaunchDescription([isaac, sim, launch_testing.actions.ReadyToTest()])


class TestSimBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # use_sim_time on the test node itself, and it is load-bearing twice
        # over: the TwistStamped headers below have to carry a stamp from the
        # same clock diff_drive_controller measures cmd_vel_timeout against,
        # or every command is judged half a second stale and the robot never
        # moves. Wall-clock stamps here would produce exactly the "no error,
        # no motion" failure this test exists to catch.
        cls.node = Node(
            'test_sim_bringup',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin(self, seconds):
        """Spin for a WALL-clock interval. Test timeouts must not use sim time.

        If /clock stops, a sim-time timeout waits forever and the failure is
        reported as a hang rather than as a missing clock.
        """
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, what):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        self.fail(f'timed out after {timeout}s waiting for {what}')

    def collect(self, topic, msg_type, timeout, qos=10):
        received = []
        sub = self.node.create_subscription(msg_type, topic, received.append, qos)
        try:
            self.wait_for(lambda: len(received) > 0, timeout, f'a message on {topic}')
            self.spin(1.0)
            return received
        finally:
            self.node.destroy_subscription(sub)

    def drive(self, linear, angular, seconds):
        """Publish a velocity command continuously and return when done.

        Continuously, because cmd_vel_timeout is 0.5 s: a single message is
        obeyed briefly and then watchdogged back to a stop.
        """
        publisher = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.addCleanup(self.node.destroy_publisher, publisher)

        end = time.time() + seconds
        while time.time() < end:
            message = TwistStamped()
            message.header.frame_id = 'base_link'
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.twist.linear.x = linear
            message.twist.angular.z = angular
            publisher.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_the_clock_comes_from_the_simulator(self):
        """/clock advances, and the test node is following it.

        First because everything else depends on it. With use_sim_time and no
        /clock, every node in the graph blocks at time zero -- no error, no log
        line, just a stack that appears hung. If this fails, nothing below it
        means anything.
        """
        messages = self.collect('/clock', Clock, timeout=60.0)
        self.wait_for(lambda: len(messages) > 5, 30.0, 'the clock to advance')

        first = messages[0].clock.sec + messages[0].clock.nanosec * 1e-9
        last = messages[-1].clock.sec + messages[-1].clock.nanosec * 1e-9
        self.assertGreater(last, first, 'clock is publishing but not advancing')

        self.assertGreater(
            self.node.get_clock().now().nanoseconds,
            0,
            'the test node is not following /clock; its use_sim_time is not set',
        )

    def test_02_the_description_selects_the_topic_based_plugin(self):
        """The sim backend differs from mock in exactly one place: the plugin."""
        from std_msgs.msg import String

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        description = self.collect('/robot_description', String, timeout=60.0, qos=qos)[-1].data
        self.assertIn('topic_based_ros2_control/TopicBasedSystem', description)
        self.assertNotIn('mock_components/GenericSystem', description)
        # The topic names the double and the graphs both hardcode.
        self.assertIn('/isaac/joint_states', description)
        self.assertIn('/isaac/joint_commands', description)

    def test_03_the_same_two_controllers_run_as_on_mock(self):
        """`ros2 control list_controllers` shows what hw:=mock shows.

        This is the acceptance criterion docs/SIM_ISAAC.md states in prose,
        asserted. Different controller names between backends would mean sim
        was testing a different stack, which is the one thing the whole design
        is arranged to prevent.
        """
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.addCleanup(self.node.destroy_client, client)
        self.wait_for(
            lambda: client.service_is_ready(), 60.0, 'the controller_manager list service'
        )

        controllers = {}
        end = time.time() + 60.0
        while time.time() < end:
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            result = future.result()
            if result is not None:
                controllers = {c.name: c for c in result.controller}
                if set(EXPECTED_CONTROLLERS) <= set(controllers) and all(
                    controllers[name].state == 'active' for name in EXPECTED_CONTROLLERS
                ):
                    break
            self.spin(1.0)

        for name, type_name in EXPECTED_CONTROLLERS.items():
            self.assertIn(name, controllers, f'loaded controllers: {sorted(controllers)}')
            self.assertEqual(controllers[name].type, type_name)
            self.assertEqual(
                controllers[name].state,
                'active',
                f'{name} is {controllers[name].state}, not active',
            )

    def test_04_commands_reach_the_simulator_as_velocities_only(self):
        """What TopicBasedSystem actually puts on /isaac/joint_commands.

        Two separate things are pinned here, and both are failures that look
        like a working stack.

        THAT ANYTHING IS PUBLISHED AT ALL. write() skips publishing when the
        position command and position state are within
        trigger_joint_command_threshold of each other. This drivetrain has no
        position command interface, so that command stays 0.0 forever; at rest
        the state is 0.0 too, and the default threshold of 1e-5 makes the skip
        permanent. No command reaches the simulator, so the wheels never turn,
        so the position never changes. lunabot.ros2_control.xacro sets the
        threshold negative to make the early return unreachable -- remove it
        and this test fails, which is the only warning there is.

        THAT THE POSITION ARRAY IS EMPTY. Isaac's ROS2SubscribeJointState
        picks position or velocity targets by which arrays are non-empty, so a
        command carrying both may drive the wheels to an ANGLE instead of a
        rate -- the rover snapping to a heading and stopping. write() pushes
        an array only for the command interfaces a joint declares, and this
        one declares velocity alone. Asserted rather than noted, because the
        day someone adds a position command interface is the day that stops
        being true.
        """
        commands = []
        sub = self.node.create_subscription(
            JointState, '/isaac/joint_commands', commands.append, 10
        )
        self.addCleanup(self.node.destroy_subscription, sub)

        self.drive(0.3, 0.0, seconds=3.0)
        self.assertTrue(
            commands,
            'TopicBasedSystem published nothing on /isaac/joint_commands. Check '
            'trigger_joint_command_threshold in lunabot.ros2_control.xacro: at the '
            'default it never publishes for a velocity-only drivetrain.',
        )

        last = commands[-1]
        self.assertTrue(
            WHEEL_JOINTS <= set(last.name),
            f'missing joints in the command: {sorted(WHEEL_JOINTS - set(last.name))}',
        )
        self.assertTrue(last.velocity, 'command carries no velocity array; the wheels get no rate')
        self.assertTrue(
            any(abs(v) > 1e-6 for v in last.velocity),
            f'all commanded velocities are zero after driving: {list(last.velocity)}',
        )
        self.assertFalse(
            list(last.position),
            'the command carries a POSITION array as well as velocity. Isaac picks '
            'position or velocity targets by which array is non-empty, so the rover '
            'will drive to an angle and stop rather than spin at a rate. Something '
            'has added a position command interface to lunabot.ros2_control.xacro, '
            'or the plugin changed.',
        )

    def test_05_odometry_responds_to_command(self):
        """The one that matters, through the sim plumbing end to end.

        /odom moving here means: the xacro expanded with hardware:=sim, the
        TopicBasedSystem plugin loaded and claimed its interfaces, commands
        went out on /isaac/joint_commands, states came back on
        /isaac/joint_states, sum_wrapped_joint_states turned the wrapped
        angles back into monotonic positions, and diff_drive_controller
        integrated them.

        If this fails while test_mock_bringup passes, the fault is in that
        chain and not in the controllers. Run isaac_double.py with
        --no-wrap-positions: if it then passes, sum_wrapped_joint_states is
        the culprit and Isaac will wrap exactly the same way.
        """
        odom = []
        sub = self.node.create_subscription(Odometry, '/odom', odom.append, 10)
        self.addCleanup(self.node.destroy_subscription, sub)

        self.wait_for(lambda: len(odom) > 0, 60.0, 'the first /odom message')
        start = odom[-1].pose.pose.position

        # Long enough for the wheels to pass the +/-pi wrap several times: at
        # 0.3 m/s on a 0.10 m wheel that is about 3 rad/s, so a wrap every two
        # seconds.
        self.drive(0.3, 0.0, seconds=5.0)

        final = odom[-1].pose.pose.position
        travelled = ((final.x - start.x) ** 2 + (final.y - start.y) ** 2) ** 0.5

        self.assertGreater(travelled, 0.3, f'robot did not move: {travelled:.3f} m')
        self.assertLess(travelled, 5.0, f'robot moved implausibly far: {travelled:.3f} m')
        self.assertGreater(final.x, start.x, 'robot moved backwards on a forward command')

    def test_06_odom_frames_match_the_contract(self):
        messages = self.collect('/odom', Odometry, timeout=30.0)
        self.assertEqual(messages[-1].header.frame_id, 'odom')
        self.assertEqual(messages[-1].child_frame_id, 'base_link')

    def test_07_ground_truth_is_published_and_stays_out_of_tf(self):
        """The decision that keeps sim honest, asserted rather than trusted.

        Ground truth exists on its own topic, in frames nothing publishes a
        transform for. If it ever entered /tf there would be two publishers on
        the same edges and a tree that works only in sim -- so every TF bug
        would stay invisible until the rover was on regolith.
        """
        truth = self.collect('/sim/ground_truth/odom', Odometry, timeout=30.0)
        self.assertEqual(truth[-1].header.frame_id, 'sim_ground_truth')
        self.assertEqual(truth[-1].child_frame_id, 'sim_base_link')

        static_qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        edges = set()
        for topic, qos in (('/tf', 10), ('/tf_static', static_qos)):
            for message in self.collect(topic, TFMessage, timeout=30.0, qos=qos):
                edges |= {(t.header.frame_id, t.child_frame_id) for t in message.transforms}

        frames = {frame for edge in edges for frame in edge}
        self.assertNotIn('sim_ground_truth', frames, 'ground truth leaked into TF')
        self.assertNotIn('sim_base_link', frames, 'ground truth leaked into TF')
        self.assertIn(('odom', 'base_link'), edges, 'nobody is publishing odom -> base_link')

    def test_08_the_contract_checker_passes_against_this_graph(self):
        """Run the tool the acceptance run uses, so the tool is tested too.

        check_stack.py is what someone types on the sim machine. It is worth
        very little if the first time it runs is the first time it is needed,
        so it runs here, against a graph whose answers are known.
        """
        result = subprocess.run(
            [
                'ros2',
                'run',
                'lunabot_bringup',
                'check_stack.py',
                '--profile',
                'sim',
                '--timeout',
                '60',
            ],
            capture_output=True,
            text=True,
            timeout=180,
        )
        print(result.stdout)
        print(result.stderr)
        self.assertEqual(result.returncode, 0, 'check_stack.py reported a contract violation')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_everything_exited_cleanly(self, proc_info):
        """A non-zero exit usually means a plugin failed to load.

        topic_based_ros2_control is built from source (lunabot.repos). If it
        is missing, controller_manager exits here rather than reporting a
        controller that never activates.

        EXIT_SIGINT is allowed alongside EXIT_OK: launch signals the graph
        down at the end of the run, and a process that exits because it was
        asked to is not a failure. (launch_testing.asserts defines EXIT_OK,
        EXIT_SIGINT, EXIT_SIGQUIT, EXIT_SIGKILL and EXIT_SIGSEGV -- there is
        no EXIT_SIGTERM, and the last three all mean something went wrong.)
        """
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[
                launch_testing.asserts.EXIT_OK,
                launch_testing.asserts.EXIT_SIGINT,
            ],
        )
