#!/usr/bin/env python3
# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Check a RUNNING stack against docs/TOPIC_FRAME_CONTRACT.md.

    ros2 run lunabot_bringup check_stack.py --profile sim

Point it at a graph that is already up. It reports one line per check and
exits non-zero if any failed, so it works as a bring-up checklist and as a
test assertion.

    --profile mock   nothing extra expected
    --profile sim    /clock, the two /isaac joint topics, ground-truth odom
    --profile real   the camera topics and /drive/status

Why this exists: the sim acceptance run used to be a paragraph of prose --
bring the stack up, check `ros2 control list_controllers` shows the same two
controllers as mock, teleop forward, watch /odom track ground truth. Every
step of that is mechanical, and a checklist a person walks through at 2am
before a competition is a checklist with steps missed in it.

What it does NOT check: whether the robot moves. That needs commanding the
drivetrain, which on hw:=real means a rover moving in a room where somebody
is holding a laptop. Use --drive to opt in; it is off by default, and the
launch tests cover motion in mock and sim.

===================== ON COUNTING /tf PUBLISHERS =====================
Two publishers on one transform is the failure this workspace is most
arranged against, and it cannot be detected from the messages: TFMessage
carries no publisher identity, so two nodes publishing odom -> base_link look
exactly like one node publishing it twice as fast.

What can be checked is WHO advertises /tf, by node name, against who is
supposed to for the profile and flags in use. That is what the tf_publishers
check does, and it is why it takes --odom-source and --slam rather than
inferring them: the expected set is a function of the launch arguments, and
guessing it would make the check unfalsifiable.
======================================================================
"""

import argparse
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage

# (topic, type) pairs from docs/TOPIC_FRAME_CONTRACT.md. Names, not shapes:
# a topic carrying the wrong type is the failure mode this catches, and it is
# a quiet one -- subscribers simply never match.
CORE_TOPICS = {
    '/robot_description': 'std_msgs/msg/String',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/odom': 'nav_msgs/msg/Odometry',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}

SIM_TOPICS = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/isaac/joint_states': 'sensor_msgs/msg/JointState',
    '/isaac/joint_commands': 'sensor_msgs/msg/JointState',
    '/sim/ground_truth/odom': 'nav_msgs/msg/Odometry',
}

REAL_TOPICS = {
    '/oak_d/rgb/image_raw': 'sensor_msgs/msg/Image',
    '/oak_d/points': 'sensor_msgs/msg/PointCloud2',
    '/oak_d/imu/data': 'sensor_msgs/msg/Imu',
    '/drive/status': 'lunabot_msgs/msg/DriveStatus',
}

EXPECTED_CONTROLLERS = {
    'joint_state_broadcaster': 'joint_state_broadcaster/JointStateBroadcaster',
    'diff_drive_controller': 'diff_drive_controller/DiffDriveController',
}

# Frames graphs/ground_truth.py publishes on its own topic. Nothing publishes
# a transform for either, and finding one in /tf means ground truth has been
# wired into the TF tree -- which makes sim flatter itself.
GROUND_TRUTH_FRAMES = {'sim_ground_truth', 'sim_base_link'}

# base_link -> * all comes from robot_state_publisher, but it arrives on two
# different topics and the split is not a detail: rsp puts FIXED joints on
# /tf_static once, latched, and MOVABLE ones on /tf every time /joint_states
# updates. Looking for a wheel on /tf_static finds nothing on a healthy stack.
REQUIRED_STATIC_CHILDREN = (
    'base_footprint',
    'oak_d_link',
    'oak_d_rgb_camera_optical_frame',
    'oak_d_imu_frame',
)

# The four continuous joints, which is why these are on /tf. Their presence
# also means /joint_states is flowing: rsp publishes them only when it has
# joint positions to publish them from.
REQUIRED_DYNAMIC_CHILDREN = (
    'front_left_wheel_link',
    'front_right_wheel_link',
    'rear_left_wheel_link',
    'rear_right_wheel_link',
)

# Who is allowed to advertise /tf, by odom_source and slam backend. The names
# come from the launch files: ekf_node in localization.launch.py,
# visual_slam_node in cuvslam.launch.py, rtabmap in rtabmap.launch.py.
#
# robot_state_publisher is in every set and is not an oversight: the wheel
# joints move, so it publishes base_link -> wheel_link on /tf rather than on
# /tf_static, and a check that treated /tf as having a single publisher would
# fail on a correct stack.
#
# controller_manager is allowed alongside diff_drive_controller because which
# of the two owns a controller's publishers is a ros2_control implementation
# detail that has moved between releases. Both names mean the same node
# process, and this check is about a SECOND owner of odom -> base_link, not
# about that detail.
TF_OWNERS = {
    'wheel': {'robot_state_publisher', 'diff_drive_controller', 'controller_manager'},
    'visual': {'robot_state_publisher', 'visual_slam_node'},
    'ekf': {'robot_state_publisher', 'ekf_node'},
}
SLAM_TF_OWNERS = {'rtabmap': {'rtabmap'}, 'cuvslam': {'visual_slam_node'}, 'none': set()}


class Result:
    """One check, its verdict, and enough detail to act on a failure."""

    def __init__(self, name, ok, detail=''):
        self.name = name
        self.ok = ok
        self.detail = detail

    def __str__(self):
        return f'[{"PASS" if self.ok else "FAIL"}] {self.name}' + (
            f'\n         {self.detail}' if self.detail else ''
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--profile',
        default='mock',
        choices=['mock', 'sim', 'real'],
        help='Which extra topics to require beyond the core contract.',
    )
    parser.add_argument(
        '--odom-source',
        default='wheel',
        choices=['wheel', 'visual', 'ekf'],
        help='Must match the odom_source the stack was launched with.',
    )
    parser.add_argument(
        '--slam',
        default='none',
        choices=['none', 'rtabmap', 'cuvslam'],
        help='Must match the slam backend the stack was launched with.',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=30.0,
        help='Seconds to wait for the graph to settle before judging it.',
    )
    parser.add_argument(
        '--drive',
        action='store_true',
        help='Also command 0.2 m/s for two seconds and check /odom responds. MOVES THE ROBOT.',
    )
    return parser.parse_args(argv)


class StackChecker(Node):
    def __init__(self, args):
        # A sim stack stamps everything from /clock. Reading those messages
        # with a wall clock makes every one of them look ancient.
        use_sim_time = args.profile == 'sim'
        super().__init__(
            'check_stack',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)],
        )
        self._args = args

    # -- helpers ------------------------------------------------------

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout):
        end = time.time() + timeout
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def topic_types(self):
        return dict(self.get_topic_names_and_types())

    def collect(self, topic, msg_type, seconds, qos=10):
        received = []
        sub = self.create_subscription(msg_type, topic, received.append, qos)
        try:
            self.spin(seconds)
            return received
        finally:
            self.destroy_subscription(sub)

    # -- checks -------------------------------------------------------

    def check_topics(self):
        expected = dict(CORE_TOPICS)
        if self._args.profile == 'sim':
            expected.update(SIM_TOPICS)
        elif self._args.profile == 'real':
            expected.update(REAL_TOPICS)

        self.wait_for(lambda: set(expected) <= set(self.topic_types()), self._args.timeout)
        present = self.topic_types()

        results = []
        for topic, type_name in sorted(expected.items()):
            if topic not in present:
                results.append(Result(f'topic {topic}', False, 'not in the graph'))
            elif type_name not in present[topic]:
                results.append(
                    Result(
                        f'topic {topic}',
                        False,
                        f'is {present[topic]}, contract says {type_name}',
                    )
                )
            else:
                results.append(Result(f'topic {topic}', True))
        return results

    def check_controllers(self):
        client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        try:
            if not self.wait_for(client.service_is_ready, self._args.timeout):
                return [
                    Result(
                        'controllers',
                        False,
                        'controller_manager is not answering; is ros2_control_node running?',
                    )
                ]

            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=self._args.timeout)
            response = future.result()
        finally:
            self.destroy_client(client)

        if response is None:
            return [Result('controllers', False, 'list_controllers did not answer')]

        loaded = {c.name: c for c in response.controller}
        results = []
        for name, type_name in EXPECTED_CONTROLLERS.items():
            if name not in loaded:
                results.append(
                    Result(f'controller {name}', False, f'loaded: {sorted(loaded) or "nothing"}')
                )
            elif loaded[name].state != 'active':
                results.append(
                    Result(f'controller {name}', False, f'state is {loaded[name].state}')
                )
            elif loaded[name].type != type_name:
                results.append(Result(f'controller {name}', False, f'type is {loaded[name].type}'))
            else:
                results.append(Result(f'controller {name}', True))
        return results

    def check_frames(self):
        static_qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        static = self.collect('/tf_static', TFMessage, 3.0, qos=static_qos)
        dynamic = self.collect('/tf', TFMessage, 3.0)

        static_edges = {
            (t.header.frame_id, t.child_frame_id) for m in static for t in m.transforms
        }
        dynamic_edges = {
            (t.header.frame_id, t.child_frame_id) for m in dynamic for t in m.transforms
        }
        all_frames = {f for edge in static_edges | dynamic_edges for f in edge}

        results = []

        static_children = {child for _, child in static_edges}
        missing = [f for f in REQUIRED_STATIC_CHILDREN if f not in static_children]
        results.append(
            Result(
                'fixed frames on /tf_static',
                not missing,
                f'no parent in /tf_static for {missing}' if missing else '',
            )
        )

        dynamic_children = {child for _, child in dynamic_edges}
        missing = [f for f in REQUIRED_DYNAMIC_CHILDREN if f not in dynamic_children]
        results.append(
            Result(
                'wheel frames on /tf',
                not missing,
                f'no transform on /tf for {missing}; robot_state_publisher publishes '
                'these from /joint_states, so silence here usually means the broadcaster '
                'is not running rather than a TF problem'
                if missing
                else '',
            )
        )

        results.append(
            Result(
                'odom -> base_link is published',
                ('odom', 'base_link') in dynamic_edges,
                ''
                if ('odom', 'base_link') in dynamic_edges
                else 'nothing owns it; check odom_source',
            )
        )

        if self._args.slam != 'none':
            results.append(
                Result(
                    'map -> odom is published',
                    ('map', 'odom') in dynamic_edges,
                    f'slam:={self._args.slam} is running but nothing publishes map -> odom',
                )
            )

        leaked = GROUND_TRUTH_FRAMES & all_frames
        results.append(
            Result(
                'ground truth stays out of TF',
                not leaked,
                f'{sorted(leaked)} found in the TF tree -- sim is now flattering itself'
                if leaked
                else '',
            )
        )
        return results

    def check_tf_publishers(self):
        expected = TF_OWNERS[self._args.odom_source] | SLAM_TF_OWNERS[self._args.slam]
        actual = {info.node_name for info in self.get_publishers_info_by_topic('/tf')}

        unexpected = actual - expected
        return [
            Result(
                'only the expected nodes advertise /tf',
                not unexpected,
                f'unexpected publisher(s) {sorted(unexpected)}; expected {sorted(expected)}. '
                'Two publishers on one transform gives a tree that looks correct in '
                'view_frames and behaves nondeterministically.'
                if unexpected
                else f'publishers: {sorted(actual)}',
            )
        ]

    def check_odometry_responds(self):
        odom = []
        sub = self.create_subscription(Odometry, '/odom', odom.append, 10)
        publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        try:
            if not self.wait_for(lambda: bool(odom), self._args.timeout):
                return [Result('odometry responds to /cmd_vel', False, 'no /odom messages at all')]
            start = odom[-1].pose.pose.position

            end = time.time() + 2.0
            while time.time() < end:
                message = TwistStamped()
                message.header.frame_id = 'base_link'
                message.header.stamp = self.get_clock().now().to_msg()
                message.twist.linear.x = 0.2
                publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.05)

            final = odom[-1].pose.pose.position
            travelled = ((final.x - start.x) ** 2 + (final.y - start.y) ** 2) ** 0.5
            return [
                Result(
                    'odometry responds to /cmd_vel',
                    travelled > 0.05,
                    f'travelled {travelled:.3f} m in 2 s at 0.2 m/s',
                )
            ]
        finally:
            self.destroy_subscription(sub)
            self.destroy_publisher(publisher)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init(args=None)
    node = StackChecker(args)

    try:
        results = []
        results += node.check_topics()
        results += node.check_controllers()
        results += node.check_frames()
        results += node.check_tf_publishers()
        if args.drive:
            results += node.check_odometry_responds()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f'\ncheck_stack: profile={args.profile} odom_source={args.odom_source} slam={args.slam}')
    print('-' * 72)
    for result in results:
        print(result)
    print('-' * 72)

    failed = [r for r in results if not r.ok]
    print(f'{len(results) - len(failed)} passed, {len(failed)} failed\n')
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
