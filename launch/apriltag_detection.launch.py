#!/usr/bin/env python3
"""
AprilTag Detection Launch File

Launches the OAK-D S2 camera with RGB stream and apriltag_ros detector.
Detected tags are published on /detections and logged to screen.

For beacon-based localization in the Lunabotics competition arena.
Fiducials can be mounted on the bin frame along the starting zone perimeter.

Usage:
    # With camera already running:
    ros2 launch lunabot_drive apriltag_detection.launch.py launch_camera:=false

    # Standalone (launches camera with RGB config):
    ros2 launch lunabot_drive apriltag_detection.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    launch_camera = LaunchConfiguration('launch_camera')
    tag_family = LaunchConfiguration('tag_family')

    default_camera_config = os.path.join(pkg_share, 'config', 'oak_d_rgb_only.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_camera', default_value='true',
            description='Launch OAK-D camera with RGB config'
        ),
        DeclareLaunchArgument(
            'tag_family', default_value='tag36h11',
            description='AprilTag family (tag36h11, tag25h9, etc.)'
        ),

        # Launch camera with RGB config (need RGB stream for AprilTag detection)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            launch_arguments={'config': default_camera_config}.items(),
            condition=IfCondition(launch_camera),
        ),

        # AprilTag detector node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[{
                'family': tag_family,
                'size': 0.16,              # Tag size in meters (adjust to actual tags)
                'max_hamming': 0,          # No error correction bits
                'detector.threads': 2,     # Pi 5 has 4 cores, use 2 for detection
                'detector.quad_decimate': 2.0,  # Downsample for speed
                'detector.quad_sigma': 0.0,
                'detector.refine_edges': True,
                'detector.decode_sharpening': 0.25,
            }],
            remappings=[
                ('image_rect', '/oak/rgb/image_raw'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
            output='screen',
        ),
    ])
