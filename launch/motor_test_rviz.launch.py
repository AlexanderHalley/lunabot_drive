#!/usr/bin/env python3
"""
Motor Test with RViz Visualization

Tests motor control with visual feedback in RViz. Shows:
  - Robot model with spinning wheels (from encoder feedback)
  - Odometry trail (odom -> base_link transform)
  - Joint states from drive_node

Usage:
    # On Pi: launch motor control
    ros2 launch lunabot_drive pi_drive.launch.py

    # On offboard PC: launch this file
    ros2 launch lunabot_drive motor_test_rviz.launch.py

    # With teleop on the same machine:
    ros2 launch lunabot_drive motor_test_rviz.launch.py enable_teleop:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    use_rviz = LaunchConfiguration('rviz')
    enable_teleop = LaunchConfiguration('enable_teleop')
    launch_drive = LaunchConfiguration('launch_drive')

    # Process the URDF xacro
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    wheel_radius = 0.1778
    wheel_base = 0.762

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument(
            'enable_teleop', default_value='false',
            description='Launch joystick teleop nodes'
        ),
        DeclareLaunchArgument(
            'launch_drive', default_value='false',
            description='Launch drive_node (set true if running everything on one machine)'
        ),

        # Robot state publisher (URDF -> TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # Static TF: map -> odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Optionally launch drive_node (with odom TF since no EKF)
        Node(
            package='lunabot_drive',
            executable='drive_node',
            name='drive_node',
            parameters=[{
                'can_interface': 'can0',
                'left_front_id': 2,
                'right_front_id': 1,
                'left_rear_id': 3,
                'right_rear_id': 4,
                'wheel_base': wheel_base,
                'wheel_radius': wheel_radius,
                'gear_ratio': 1.0,
                'max_duty_cycle': 0.8,
                'joint_state_rate': 50.0,
                'publish_odom_tf': True,
            }],
            condition=IfCondition(launch_drive),
            output='screen',
        ),

        # Joystick teleop (optional)
        GroupAction(
            condition=IfCondition(enable_teleop),
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy_node',
                    parameters=[{'device_id': 0}],
                ),
                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='teleop_twist_joy_node',
                    parameters=[
                        os.path.join(pkg_share, 'config', 'switch_pro.yaml')
                    ],
                ),
            ],
        ),

        # RViz2 (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'rviz', 'oak_d_camera.rviz')],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
