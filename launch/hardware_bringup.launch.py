#!/usr/bin/env python3
"""
Unified Hardware Bringup Launch File

Starts all hardware nodes on the Raspberry Pi:
  1. SparkFlex CAN motor driver (drive_node)
  2. OAK-D S2 camera with pointcloud config
  3. robot_state_publisher (URDF -> TF tree)
  4. robot_localization EKF (fuses wheel odom + camera IMU)
  5. Joystick teleop (optional, for manual control)

Usage:
    ros2 launch lunabot_drive hardware_bringup.launch.py
    ros2 launch lunabot_drive hardware_bringup.launch.py enable_teleop:=false
    ros2 launch lunabot_drive hardware_bringup.launch.py use_ekf:=false
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    # Launch arguments
    enable_teleop = LaunchConfiguration('enable_teleop')
    use_ekf = LaunchConfiguration('use_ekf')
    camera_config = LaunchConfiguration('camera_config')

    # Default paths
    default_camera_config = os.path.join(
        pkg_share, 'config', 'oak_d_pointcloud_only.yaml'
    )
    ekf_params = os.path.join(pkg_share, 'config', 'params', 'ekf_params.yaml')

    # Process the URDF xacro
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Wheel parameters from URDF (description/robot_core.xacro)
    wheel_radius = 0.1778   # 7 inches
    wheel_base = 0.52      # distance between left and right wheels

    return LaunchDescription([
        # ── Launch Arguments ──
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='true',
            description='Enable joystick teleop nodes'
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run robot_localization EKF (fuses wheel odom + IMU)'
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value=default_camera_config,
            description='Camera configuration YAML file'
        ),

        # ── 1. Motor Drive Node ──
        # When EKF is running, drive_node should NOT publish odom->base_link TF
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
                'gear_ratio': 100.0,
                'max_duty_cycle': 0.8,
                'joint_state_rate': 50.0,
                'publish_odom_tf': False,  # EKF handles this by default
            }],
            condition=IfCondition(use_ekf),
            output='screen',
        ),
        # When EKF is NOT running, drive_node publishes odom->base_link TF itself
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
            condition=UnlessCondition(use_ekf),
            output='screen',
        ),

        # ── 2. OAK-D S2 Camera ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            launch_arguments={'config': camera_config}.items(),
        ),

        # ── 3. Robot State Publisher (URDF -> TF) ──
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── 4. robot_localization EKF ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            condition=IfCondition(use_ekf),
            output='screen',
        ),

        # ── 5. Joystick Teleop (optional) ──
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
    ])
