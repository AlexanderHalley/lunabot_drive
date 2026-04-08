#!/usr/bin/env python3
"""
Camera-Only Nav2 Testing Launch File

Tests depth camera -> costmap pipeline without motors or LIDAR.
Creates a minimal TF tree with static transforms for a stationary robot.

Usage:
    # Terminal 1 (Pi): Launch camera
    ros2 launch lunabot_drive oak_d_camera.launch.py

    # Terminal 2 (Offboard): Launch this file
    ros2 launch lunabot_drive camera_only_nav2.launch.py

    # Terminal 3 (Offboard): Visualize
    rviz2
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory
import lifecycle_msgs.msg


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    launch_camera = LaunchConfiguration('launch_camera')

    # Config files
    nav2_params = os.path.join(pkg_share, 'config', 'camera_only_nav2.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'camera_only_nav2.rviz')

    # Camera mount position (from URDF: front of chassis, on top)
    # These values are from description/robot_core.xacro and oak_d_s2.xacro
    # chassis_length = 0.9652, chassis_height = 0.138, camera mounted at front
    camera_x = '0.48'   # Relative to base_link (center of robot)
    camera_y = '0.0'
    camera_z = '0.15'   # chassis_height + camera mount height

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2'
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='false',
            description='Launch camera node (set true if running everything on one machine)'
        ),

        # Optionally launch camera (if testing on single machine)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            condition=IfCondition(launch_camera)
        ),

        # Static TF: map -> odom (identity - no localization)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # Static TF: odom -> base_link (identity - stationary robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # Static TF: base_link -> oak (camera mount position)
        # depthai_ros_driver publishes oak -> optical frames
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_camera',
            arguments=[camera_x, camera_y, camera_z, '0', '0', '0', 'base_link', 'oak'],
        ),

        # Local costmap node
        LifecycleNode(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            namespace='',
            output='screen',
            parameters=[nav2_params],
        ),

        # Lifecycle manager to auto-start the costmap
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['local_costmap'],
            }],
        ),

        # RViz2 (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
    ])
