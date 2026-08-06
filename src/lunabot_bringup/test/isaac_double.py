#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A stand-in for Isaac Sim's ROS surface, for testing the hw:=sim path.

    python3 src/lunabot_bringup/test/isaac_double.py &
    ros2 launch lunabot_bringup sim.launch.py rviz:=false teleop:=false

Isaac's OmniGraphs put five things on the ROS graph. This publishes the three
that the control stack cannot come up without, and nothing else:

    /clock                     rosgraph_msgs/Clock     graphs/clock.py
    /isaac/joint_states        sensor_msgs/JointState  graphs/joints.py
    /sim/ground_truth/odom     nav_msgs/Odometry       graphs/ground_truth.py

and subscribes to /isaac/joint_commands, exactly as Isaac's
ROS2SubscribeJointState node does. Camera and IMU are left out: they need a
renderer, and nothing in the control loop waits on them.

==================== WHAT THIS IS AND IS NOT ====================
It is a TEST FIXTURE. It lives in test/, is not installed, is not launched by
anything in launch/, and no part of the robot may depend on it. The rule in
CONTRIBUTING.md against sim-only nodes is about the STACK -- a double that
stands in for hardware during a test is the ordinary way to test a hardware
path, and this one exists precisely so that hw:=sim is not first exercised on
the day the GPU machine arrives.

It is NOT a simulator. There is no contact, no friction, no mass: wheels
achieve commanded velocity instantly and the ground truth is a perfect
differential integration of the commands. That makes it a test of the ROS
plumbing -- topic names, message types, use_sim_time, TF ownership, whether
TopicBasedSystem and diff_drive_controller agree about anything -- and no
test at all of whether the rover can climb a slope. Isaac answers that.
================================================================

Two things it deliberately reproduces, because both are places the real thing
is expected to hurt:

  1. WRAPPED JOINT POSITIONS. Isaac reports a revolute joint's angle in
     [-pi, pi]; a wheel driving forwards crosses that boundary every couple of
     seconds. `sum_wrapped_joint_states: true` in lunabot.ros2_control.xacro
     is what turns those back into the monotonic position
     diff_drive_controller integrates odometry from. If that parameter ever
     stops working, /odom goes backwards twice per revolution, and this is
     where it shows up. `--no-wrap-positions` turns the emulation off, which
     is the first thing to try when the sim bringup test fails and the mock
     one passes.

  2. STARTING BEFORE THE ROS GRAPH. Isaac must publish /clock first or every
     node with use_sim_time blocks at time zero. This publishes /clock from
     its first spin, so sim.launch.py's wait-for-clock barrier is exercised
     rather than skipped.
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

# Must match lunabot_description/urdf/lunabot.urdf.xacro and the
# left_wheel_names/right_wheel_names in controllers.yaml.
WHEEL_JOINTS = (
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
)
LEFT_JOINTS = ('front_left_wheel_joint', 'rear_left_wheel_joint')
RIGHT_JOINTS = ('front_right_wheel_joint', 'rear_right_wheel_joint')

# The topic names in graphs/joints.py and graphs/ground_truth.py, which are
# themselves the defaults of sim_joint_states_topic and sim_joint_commands_topic
# in the xacro. All three have to agree; see docs/TOPIC_FRAME_CONTRACT.md.
STATES_TOPIC = '/isaac/joint_states'
COMMANDS_TOPIC = '/isaac/joint_commands'
GROUND_TRUTH_TOPIC = '/sim/ground_truth/odom'

# graphs/ground_truth.py publishes these, and NOTHING publishes a transform
# for either. That is what keeps ground truth out of the TF tree.
GROUND_TRUTH_FRAME = 'sim_ground_truth'
GROUND_TRUTH_CHILD = 'sim_base_link'


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--rate', type=float, default=100.0, help='Publish rate, Hz. Isaac runs the graphs at 60.'
    )
    parser.add_argument(
        '--wheel-radius',
        type=float,
        default=0.10,
        help='Only for the ground-truth integration. Keep it equal to controllers.yaml.',
    )
    parser.add_argument(
        '--wheel-separation',
        type=float,
        default=0.50,
        help='Only for the ground-truth integration. Keep it equal to controllers.yaml.',
    )
    parser.add_argument(
        '--no-wrap-positions',
        dest='wrap_positions',
        action='store_false',
        help=(
            'Report unwrapped joint positions. Isaac does NOT do this -- see the '
            'module docstring. Useful only to find out whether a failure is '
            'sum_wrapped_joint_states.'
        ),
    )
    return parser.parse_args(argv)


class IsaacDouble(Node):
    """Publishes what Isaac's bridge publishes, driven by a wall-clock timer."""

    def __init__(self, args):
        # use_sim_time false, and it is not boilerplate: this node IS the
        # clock. A clock source that waits for a clock never starts.
        super().__init__(
            'isaac_double',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, False)],
        )

        self._args = args
        self._dt = 1.0 / args.rate
        # Integer nanoseconds, not accumulated floats: /clock is the time
        # source for the whole graph, and a clock that drifts against its own
        # step size is a miserable thing to debug on top of everything else.
        self._step_nanos = int(round(1e9 / args.rate))
        self._sim_nanos = 0
        self._positions = dict.fromkeys(WHEEL_JOINTS, 0.0)
        self._velocities = dict.fromkeys(WHEEL_JOINTS, 0.0)
        self._pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self._twist = (0.0, 0.0)  # linear, angular, for the ground-truth message

        # Isaac's publishers are plain reliable/volatile. Matching that here
        # keeps the double from papering over a QoS mismatch that would bite
        # against the real bridge.
        qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._clock_pub = self.create_publisher(Clock, '/clock', qos)
        self._states_pub = self.create_publisher(JointState, STATES_TOPIC, qos)
        self._truth_pub = self.create_publisher(Odometry, GROUND_TRUTH_TOPIC, qos)
        self.create_subscription(JointState, COMMANDS_TOPIC, self._on_command, qos)

        self.create_timer(self._dt, self._step)
        self.get_logger().info(
            f'isaac_double publishing /clock, {STATES_TOPIC} and {GROUND_TRUTH_TOPIC} '
            f'at {args.rate:g} Hz (wrapped positions: {args.wrap_positions})'
        )

    def _on_command(self, msg):
        """Take velocity targets, exactly as ROS2SubscribeJointState does.

        Position targets are ignored on purpose. Isaac's articulation drives
        are configured with zero stiffness (robot/articulation.py) so they
        cannot hold a position either, and positionCommand is left unwired in
        graphs/joints.py. A double that obeyed a stray position array would
        hide the very failure those two defences exist for.
        """
        if not msg.velocity:
            return
        for index, name in enumerate(msg.name):
            if name in self._velocities and index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])

    def _step(self):
        self._sim_nanos += self._step_nanos
        for name, velocity in self._velocities.items():
            self._positions[name] += velocity * self._dt
        self._integrate_ground_truth()

        stamp = Time(nanoseconds=self._sim_nanos).to_msg()
        self._clock_pub.publish(Clock(clock=stamp))
        self._states_pub.publish(self._joint_state(stamp))
        self._truth_pub.publish(self._ground_truth(stamp))

    def _integrate_ground_truth(self):
        """Differential kinematics from the wheel velocities.

        This is the trajectory Isaac would produce with infinite traction. The
        point of publishing it is not the numbers -- it is that /odom and
        /sim/ground_truth/odom exist side by side, in the frames the contract
        names, so the comparison the acceptance run makes is wired before the
        GPU arrives.
        """
        left = sum(self._velocities[j] for j in LEFT_JOINTS) / len(LEFT_JOINTS)
        right = sum(self._velocities[j] for j in RIGHT_JOINTS) / len(RIGHT_JOINTS)

        linear = (left + right) * 0.5 * self._args.wheel_radius
        angular = (right - left) * self._args.wheel_radius / self._args.wheel_separation

        self._pose[2] += angular * self._dt
        self._pose[0] += linear * math.cos(self._pose[2]) * self._dt
        self._pose[1] += linear * math.sin(self._pose[2]) * self._dt
        self._twist = (linear, angular)

    def _joint_state(self, stamp):
        message = JointState()
        message.header.stamp = stamp
        message.name = list(WHEEL_JOINTS)
        message.position = [self._report(self._positions[j]) for j in WHEEL_JOINTS]
        message.velocity = [self._velocities[j] for j in WHEEL_JOINTS]
        return message

    def _report(self, position):
        """Wrap into [-pi, pi] the way a revolute joint's angle reads."""
        if not self._args.wrap_positions:
            return position
        return math.remainder(position, 2.0 * math.pi)

    def _ground_truth(self, stamp):
        linear, angular = self._twist
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = GROUND_TRUTH_FRAME
        message.child_frame_id = GROUND_TRUTH_CHILD
        message.pose.pose.position.x = self._pose[0]
        message.pose.pose.position.y = self._pose[1]
        message.pose.pose.orientation = Quaternion(
            z=math.sin(self._pose[2] / 2.0), w=math.cos(self._pose[2] / 2.0)
        )
        message.twist.twist.linear.x = linear
        message.twist.twist.angular.z = angular
        return message


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = IsaacDouble(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
