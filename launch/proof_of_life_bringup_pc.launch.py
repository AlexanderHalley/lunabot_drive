#!/usr/bin/env python3
"""
Proof-of-life bringup for dreamfyre (MCC laptop).

Starts everything the PC needs for a manual teleop run:
  1. joy_node             — reads Switch Pro controller (USB or Bluetooth)
  2. teleop_twist_joy     — maps left stick to /cmd_vel for drive
  3. bucket_teleop_node   — maps d-pad to bucket lift/tilt actuator commands

The Switch Pro controller must be connected to dreamfyre before launching.
Verify with: ls /dev/input/js*

Pre-requisites on dreamfyre:
  - ROS_DOMAIN_ID=42 set in ~/.bashrc
  - lunapi reachable on the network: ping lunapi
  - proof_of_life_bringup_pi.launch.py already running on lunapi

Usage:
    ros2 launch lunabot_drive proof_of_life_bringup_pc.launch.py
    ros2 launch lunabot_drive proof_of_life_bringup_pc.launch.py joy_device_id:=1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    joy_device_id = LaunchConfiguration('joy_device_id')

    return LaunchDescription([

        DeclareLaunchArgument(
            'joy_device_id',
            default_value='0',
            description='JS device index for the Switch Pro controller (/dev/input/jsN)',
        ),

        # ── 1. Joystick driver ─────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': joy_device_id}],
            output='screen',
        ),

        # ── 2. Drive teleop (left stick -> /cmd_vel) ───────────────────────
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[
                os.path.join(pkg_share, 'config', 'switch_pro.yaml')
            ],
            output='screen',
        ),

        # ── 3. Bucket teleop (d-pad -> lift/tilt actuator commands) ────────
        Node(
            package='lunabot_drive',
            executable='bucket_teleop_node',
            name='bucket_teleop_node',
            output='screen',
        ),
    ])
