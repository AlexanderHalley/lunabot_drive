#!/usr/bin/env python3
"""
SLAM Bring-Up — Pi side (run on lunapi).

Reuses hardware_bringup.launch.py (drive_node + camera + RSP + EKF + teleop)
and adds:

  - pointcloud_to_laserscan  — projects /oak/stereo/points → /scan
  - async_slam_toolbox_node  — builds occupancy /map, publishes map->odom

This launch is INCOMPATIBLE with autonomy_bringup_pi.launch.py — both
slam_toolbox and apriltag_localizer want to publish map->odom. Use this
stack for exploration / pre-match map capture; switch to autonomy_bringup_pi
for tag-localized runs.

Usage:
    ros2 launch lunabot_drive slam_bringup_pi.launch.py
    ros2 launch lunabot_drive slam_bringup_pi.launch.py enable_teleop:=false

Save the map afterwards (from any terminal on the same domain):
    ros2 run nav2_map_server map_saver_cli -f ~/maps/arena
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    enable_teleop = LaunchConfiguration('enable_teleop')
    camera_config = LaunchConfiguration('camera_config')

    default_camera_config = os.path.join(
        pkg_share, 'config', 'oak_d_pointcloud_only.yaml'
    )
    slam_params = os.path.join(
        pkg_share, 'config', 'params', 'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_teleop', default_value='true',
            description='Forwarded to hardware_bringup (joystick teleop on/off)'
        ),
        DeclareLaunchArgument(
            'camera_config', default_value=default_camera_config,
            description='OAK-D config YAML — must publish /oak/stereo/points'
        ),

        # ── Hardware: drive + camera + RSP + EKF + (optional) teleop ──
        # use_ekf is forced true so the EKF (not drive_node) publishes
        # odom->base_link, which is what slam_toolbox reads.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'hardware_bringup.launch.py'
                ])
            ]),
            launch_arguments={
                'enable_teleop': enable_teleop,
                'use_ekf': 'true',
                'camera_config': camera_config,
            }.items(),
        ),

        # ── Project pointcloud to 2D laser scan ──
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.05,
                'min_height': 0.05,
                'max_height': 0.5,
                'angle_min': -1.5708,      # -90 deg
                'angle_max':  1.5708,      # +90 deg (camera horizontal FOV)
                'angle_increment': 0.0087, # ~0.5 deg
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
            remappings=[
                ('cloud_in', '/oak/stereo/points'),
                ('scan',     '/scan'),
            ],
            output='screen',
        ),

        # ── slam_toolbox (online async) ──
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params],
            output='screen',
        ),
    ])
