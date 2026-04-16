#!/usr/bin/env python3
"""
Autonomy Bringup — PC side (run on dreamfyre).

Provides joystick teleop during both the manual phases of a competition run
and as a safety override during autonomy.  cmd_vel is published to the
joystick input of cmd_vel_mux on lunapi — that mux gives joystick priority 10
vs Nav2 priority 1, so picking up the controller immediately pre-empts
any active navigation goal.

Also launches bucket_teleop_node so the d-pad can command actuators manually.

Stack launched here
-------------------
  1. joy_node            — reads Switch Pro controller
  2. teleop_twist_joy    — maps sticks → /cmd_vel_mux/input/joystick (over LAN)
  3. bucket_teleop_node  — d-pad → /bucket/lift_mux/input/teleop, /bucket/tilt_mux/input/teleop

NOT launched here (all run on lunapi via autonomy_bringup_pi.launch.py):
  - drive_node, camera, EKF, Nav2, apriltag, mission_state_node, rosbridge, etc.

Pre-requisites on dreamfyre
---------------------------
  - ROS_DOMAIN_ID=42 set in ~/.bashrc
  - lunapi reachable:   ping lunapi
  - autonomy_bringup_pi.launch.py already running on lunapi
  - Switch Pro controller connected (USB or Bluetooth)
    Verify: ls /dev/input/js*

Usage
-----
  ros2 launch lunabot_drive autonomy_bringup_pc.launch.py
  ros2 launch lunabot_drive autonomy_bringup_pc.launch.py joy_device_id:=1

Dashboard
---------
  Open dashboard/dashboard.html in Chromium after launching the Pi side:
    chromium-browser file://<path>/dashboard/dashboard.html

RViz2 (optional, for map/pose/costmap visualisation)
-----
  ros2 launch lunabot_drive autonomy_bringup_pc.launch.py  # then separately:
  rviz2 -d <rviz_config>
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('lunabot_drive')
    teleop_config = os.path.join(pkg, 'config', 'switch_pro.yaml')

    joy_device_id = LaunchConfiguration('joy_device_id')

    return LaunchDescription([

        DeclareLaunchArgument(
            'joy_device_id',
            default_value='0',
            description='JS device index for the Switch Pro controller (/dev/input/jsN)',
        ),

        # ── 1. Joystick driver ────────────────────────────────────────────────
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': joy_device_id}],
            output='screen',
        ),

        # ── 2. Drive teleop → cmd_vel_mux joystick input on lunapi ───────────
        #    remapped to /cmd_vel_mux/input/joystick so the mux on the Pi
        #    can arbitrate between this and Nav2's cmd_vel.
        #    Priority 10 in the mux means joystick always wins when active.
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[teleop_config],
            remappings=[('cmd_vel', '/cmd_vel_mux/input/joystick')],
            output='screen',
        ),

        # ── 3. Bucket d-pad teleop ────────────────────────────────────────────
        #    Publishes Float64 to /lift_actuator/command and /tilt_actuator/command.
        #    These topics travel over LAN to the actuator driver nodes on lunapi.
        Node(
            package='lunabot_drive',
            executable='bucket_teleop_node',
            name='bucket_teleop_node',
            output='screen',
        ),
    ])
