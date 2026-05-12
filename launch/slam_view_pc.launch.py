#!/usr/bin/env python3
"""
SLAM view — PC side (dreamfyre).

Launches RViz with the SLAM config (Map + LaserScan + RobotModel + TF + Image).
Assumes the Pi is already running `slam_bringup_pi.launch.py` and publishing
/map, /scan, /tf, and the camera topics.

Usage:
    ros2 launch lunabot_drive slam_view_pc.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    default_rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'rviz', 'slam.rviz'
    ])

    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_config,
            description='Path to RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
