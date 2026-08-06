# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Start controller_manager and spawn the controllers.

Identical for mock, sim and real hardware. The hardware plugin is chosen by
the URDF (see description.launch.py), not here -- this file does not know or
care which one is loaded, which is exactly the property that makes sim a real
test of the robot's control stack.

Spawn order matters: joint_state_broadcaster first, then diff_drive_controller.
The broadcaster claims every state interface; the diff drive controller claims
the velocity command interfaces. Spawning them concurrently is a race that
usually works and occasionally leaves a controller inactive with a message
about unavailable interfaces.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'controllers_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_bringup'), 'config', 'controllers.yaml']
        ),
        description='Controller parameter file.',
    ),
    DeclareLaunchArgument(
        'enable_odom_tf',
        default_value='true',
        description=(
            'Whether diff_drive_controller publishes odom -> base_link. '
            'Set false when another node owns it (odom_source:=visual|ekf).'
        ),
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    # controller_manager takes the robot description from the
    # /robot_description topic published by robot_state_publisher, rather than
    # from a parameter. That is the Jazzy-and-later convention and it means the
    # description is expanded exactly once per launch instead of once per node
    # that needs it -- so the two copies cannot drift.
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='both',
        parameters=[
            LaunchConfiguration('controllers_file'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            # diff_drive_controller subscribes on ~/cmd_vel, i.e.
            # /diff_drive_controller/cmd_vel. The contract says the drivetrain
            # listens on /cmd_vel, with twist_mux upstream of it.
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
            ('/diff_drive_controller/odom', '/odom'),
        ],
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager',
            '/controller_manager',
            '--param-file',
            LaunchConfiguration('controllers_file'),
            # Overrides the YAML so odom TF ownership is a launch-time
            # decision. Only one node may publish odom -> base_link.
            '--controller-ros-args',
            ['-p enable_odom_tf:=', LaunchConfiguration('enable_odom_tf')],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(
        ARGUMENTS
        + [
            controller_manager,
            joint_state_broadcaster,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_state_broadcaster,
                    on_exit=[diff_drive_controller],
                )
            ),
        ]
    )
