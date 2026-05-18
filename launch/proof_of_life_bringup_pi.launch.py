#!/usr/bin/env python3
"""
Proof-of-life bringup for lunapi (Raspberry Pi 5).

Starts everything the Pi needs for a manual teleop run:
  1. drive_node         — SparkFlex CAN motors, publishes /odom and odom->base_link TF
  2. robot_state_publisher — URDF -> TF tree
  3. bucket actuators   — lift + tilt ActuatorDriverNode instances
  4. mission_state_node — autonomy state machine (dashboard Panel 1)
  5. bandwidth_monitor_node — WiFi throughput monitor
  6. health_monitor_node    — node liveness checker
  7. wifi_monitor_node      — RSSI monitor
  8. rosbridge_websocket    — WebSocket bridge for dashboard.html on dreamfyre

Arguments
---------
  enable_drive:=true   (default)  Include drive_node — requires CAN bus (can0) to be up.
                                  Run once on boot: sudo ./scripts/initialise_can
  enable_drive:=false             Skip drive_node — use for GUI / actuator-only testing
                                  when motors are not connected.

Usage:
    # Full hardware (CAN + actuators):
    ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py

    # GUI + actuators only, no CAN/motors:
    ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py enable_drive:=false

    # Add OAK-D camera to either of the above:
    ros2 launch lunabot_drive proof_of_life_bringup_pi.launch.py enable_camera:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    enable_drive  = LaunchConfiguration('enable_drive')
    enable_camera = LaunchConfiguration('enable_camera')

    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    wheel_radius = 0.1778   # 7 in radius (14 in diameter)
    wheel_base   = 0.762    # distance between left and right wheels (2 * wheel_offset_y)

    return LaunchDescription([

        DeclareLaunchArgument(
            'enable_drive',
            default_value='true',
            description='Launch drive_node (requires CAN bus / can0). '
                        'Set false for GUI or actuator-only testing without motors.',
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='false',
            description='Launch OAK-D S2 camera node (adds ~0.5 Mbps to bandwidth)',
        ),

        # ── 1. Drive node (open-loop, no EKF) ──────────────────────────────
        # Skipped when enable_drive:=false — CAN bus not required in that mode.
        Node(
            condition=IfCondition(enable_drive),
            package='lunabot_drive',
            executable='drive_node',
            name='drive_node',
            parameters=[{
                'can_interface':    'can0',
                'left_front_id':    2,
                'right_front_id':   3,
                'left_rear_id':     1,
                'right_rear_id':    4,
                'wheel_base':       wheel_base,
                'wheel_radius':     wheel_radius,
                'gear_ratio':       100.0,
                'max_duty_cycle':   0.8,
                'joint_state_rate': 50.0,
                'publish_odom_tf':  True,   # no EKF, drive_node owns odom->base_link
            }],
            output='screen',
        ),

        # ── 2. Robot state publisher (URDF -> TF) ──────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── 3. Bucket actuators (lift + tilt) ──────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'bucket_bringup.launch.py'])
            ),
        ),

        # ── 4. OAK-D S2 camera (optional) ─────────────────────────────────
        GroupAction(
            condition=IfCondition(enable_camera),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
                    ),
                ),
            ],
        ),

        # ── 5. Dashboard support nodes ─────────────────────────────────────
        Node(
            package='lunabot_drive',
            executable='mission_state_node',
            name='mission_state_node',
            output='screen',
        ),
        Node(
            package='lunabot_drive',
            executable='bandwidth_monitor_node',
            name='bandwidth_monitor_node',
            output='screen',
        ),
        Node(
            package='lunabot_drive',
            executable='health_monitor_node',
            name='health_monitor_node',
            output='screen',
        ),
        Node(
            package='lunabot_drive',
            executable='wifi_monitor_node',
            name='wifi_monitor_node',
            output='screen',
        ),

        # ── 6. rosbridge (dashboard.html connects here) ────────────────────
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            parameters=[{'port': 9090}],
            output='screen',
        ),
    ])
