#!/usr/bin/env python3
"""
Camera Mapping Test Launch File

Tests the OAK-D S2 camera pointcloud pipeline with a pointcloud_to_laserscan
bridge, suitable for initial mapping experiments.

The robot is stationary (static TFs), so this is for verifying:
  - Pointcloud publishes correctly
  - pointcloud_to_laserscan produces scan data
  - Visualization works in RViz

Usage:
    # On Pi: launch camera
    ros2 launch lunabot_drive oak_d_camera.launch.py \
        config:=$(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/oak_d_pointcloud_only.yaml

    # On offboard PC:
    ros2 launch lunabot_drive camera_mapping_test.launch.py

    # Single machine (with camera attached):
    ros2 launch lunabot_drive camera_mapping_test.launch.py launch_camera:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    use_rviz = LaunchConfiguration('rviz')
    launch_camera = LaunchConfiguration('launch_camera')
    use_rsp = LaunchConfiguration('use_rsp')

    # Process the URDF xacro
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    default_camera_config = os.path.join(
        pkg_share, 'config', 'oak_d_pointcloud_only.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument(
            'launch_camera', default_value='false',
            description='Launch camera node (true for single-machine testing)'
        ),
        DeclareLaunchArgument(
            'use_rsp', default_value='false',
            description='Launch robot_state_publisher locally. Set false when Pi is already running hardware_bringup (RSP runs there)'
        ),

        # Optionally launch camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            launch_arguments={'config': default_camera_config}.items(),
            condition=IfCondition(launch_camera),
        ),

        # Robot state publisher — disabled by default when Pi is already running
        # hardware_bringup.launch.py (which has its own RSP on the network).
        # Enable with use_rsp:=true for single-machine testing.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
            condition=IfCondition(use_rsp),
        ),

        # Static TFs and joint_state_publisher — only needed in standalone mode.
        # When the Pi is running hardware_bringup, the EKF publishes odom->base_link
        # dynamically and drive_node publishes joint states; these would conflict.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            condition=IfCondition(use_rsp),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            condition=IfCondition(use_rsp),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=IfCondition(use_rsp),
        ),

        # Pointcloud to LaserScan bridge (for SLAM toolbox compatibility)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': 0.05,       # Ignore ground plane
                'max_height': 0.5,        # Ignore above robot height
                'angle_min': -1.5708,     # -90 degrees
                'angle_max': 1.5708,      # +90 degrees (camera FOV)
                'angle_increment': 0.0087, # ~0.5 degree resolution
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/oak/stereo/points'),
                ('scan', '/scan'),
            ],
        ),

        # RViz2 (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
