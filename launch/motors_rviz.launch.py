#!/usr/bin/env python3
"""
Motor Visualization Launch File
================================

This launch file provides visualization of the robot's motor/wheel positions in RViz2
by combining motor control with the robot URDF model.

What this launches:
-------------------
1. drive_node - Motor control node that:
   - Controls the 4 wheel motors via CAN bus
   - Publishes joint_states for wheel positions
   - Subscribes to /cmd_vel for velocity commands

2. robot_state_publisher - Publishes robot TF transforms:
   - Reads robot.urdf.xacro to get robot structure
   - Uses joint_states from drive_node to update wheel positions
   - Publishes TF tree for visualization

3. rviz2 (optional) - 3D visualization:
   - Shows robot model with moving wheels
   - Displays coordinate frames
   - Can be disabled with rviz:=false

Usage:
------
Typical Setup (motors on Pi, RViz on PC):
    # On Raspberry Pi (publishes joint states):
    export ROS_DOMAIN_ID=42
    ros2 launch lunabot_drive motors_rviz.launch.py

    # On PC (for visualization):
    export ROS_DOMAIN_ID=42
    ros2 run rviz2 rviz2 -d $(ros2 pkg prefix lunabot_drive)/share/lunabot_drive/config/rviz/oak_d_camera.rviz

Single Machine with Display (if Pi has display):
    ros2 launch lunabot_drive motors_rviz.launch.py rviz:=true

With Teleop Control:
    # Terminal 1 (PC): Launch joystick + teleop
    ros2 launch lunabot_drive pc_teleop.launch.py

    # Terminal 2 (Pi): Launch motors + visualization
    ros2 launch lunabot_drive motors_rviz.launch.py

Launch Arguments:
-----------------
- rviz:=true|false
    Enable/disable RViz2 visualization (default: false)
    Note: Pi is typically headless, so RViz runs on PC

- rviz_config:=<path>
    Path to custom RViz config file
    (default: config/rviz/oak_d_camera.rviz)

Prerequisites:
--------------
- CAN interface must be up: sudo ip link set can0 up type can
- SparkFlex motor controllers must be connected to CAN bus
- Motor IDs must match configuration (1-4 by default)
- For network operation: ROS_DOMAIN_ID must match on all machines

Topics Published:
-----------------
- /joint_states (sensor_msgs/JointState) - Wheel positions and velocities
- /tf, /tf_static (tf2_msgs/TFMessage) - Robot coordinate transforms

Topics Subscribed:
------------------
- /cmd_vel (geometry_msgs/Twist) - Velocity commands for motors

Nodes:
------
- /drive_node - Motor control and joint state publishing
- /robot_state_publisher - URDF to TF transform publisher
- /rviz2 - 3D visualization (if enabled)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    # Default RViz config
    default_rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'rviz', 'oak_d_camera.rviz'
    ])

    # Process the URDF xacro
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Wheel parameters (must match URDF and drive_node)
    wheel_radius = 0.1778        # 7 inches
    wheel_base = 0.762           # distance between left and right wheels

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization (default: false, run RViz on PC instead)'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz config file'
        ),

        # Drive node (motor control + joint state publishing)
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
                'joint_state_rate': 50.0,   # Hz - rate to publish joint states
            }],
            output='screen'
        ),

        # Robot state publisher (publishes TF from URDF using joint states)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
            }],
            output='screen'
        ),

        # RViz2 node (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
