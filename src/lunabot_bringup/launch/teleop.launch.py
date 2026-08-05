# Copyright 2027 Lunabot. Licensed under the MIT License.

"""Joystick teleoperation and velocity arbitration.

    joy_node -> /joy -> teleop_twist_joy -> /cmd_vel_joy ---.
                                                             +-> twist_mux -> /cmd_vel
    nav2 --------------------------------> /cmd_vel_nav ----'

twist_mux is new for 2027. The 2026 robot had teleop publishing straight to
/cmd_vel, which works exactly until a second command source exists -- and then
"whoever published most recently wins" is the arbitration policy, which is not
a policy.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        'joy',
        default_value='true',
        description='Start joy_node and teleop_twist_joy. False leaves twist_mux running alone.',
    ),
    DeclareLaunchArgument(
        'joy_device_id',
        default_value='0',
        description='Index into /dev/input/js*.',
    ),
    DeclareLaunchArgument(
        'teleop_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_bringup'), 'config', 'teleop_switch_pro.yaml']
        ),
        description='Controller mapping. Swap this file for a different gamepad.',
    ),
    DeclareLaunchArgument(
        'twist_mux_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_bringup'), 'config', 'twist_mux.yaml']
        ),
        description='Velocity source priorities and timeouts.',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    joy_enabled = IfCondition(LaunchConfiguration('joy'))

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        condition=joy_enabled,
        parameters=[
            {
                'device_id': LaunchConfiguration('joy_device_id'),
                # Ignore stick noise around centre. Without this, a worn
                # controller trickles nonzero commands and twist_mux never
                # sees teleop release the bus, so navigation can never take
                # over.
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
                'use_sim_time': use_sim_time,
            }
        ],
        output='screen',
    )

    teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        condition=joy_enabled,
        parameters=[
            LaunchConfiguration('teleop_config'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/cmd_vel', '/cmd_vel_joy')],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[
            LaunchConfiguration('twist_mux_config'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    return LaunchDescription(ARGUMENTS + [joy_node, teleop, twist_mux])
