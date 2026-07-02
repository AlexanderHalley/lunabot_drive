"""Joystick teleop -> /cmd_vel_joy (arbitrated by twist_mux).

    ros2 launch lunabot_control teleop.launch.py

Carried over from the 2026 pc_teleop.launch.py, retargeted so teleop is one
prioritised input into twist_mux rather than driving /cmd_vel directly.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joystick_yaml = PathJoinSubstitution([
        FindPackageShare("lunabot_control"), "config", "joystick.yaml"
    ])
    return LaunchDescription([
        Node(package="joy", executable="joy_node",
             parameters=[{"device_id": 0}]),
        Node(package="teleop_twist_joy", executable="teleop_node",
             name="teleop_twist_joy_node",
             parameters=[joystick_yaml],
             remappings=[("cmd_vel", "cmd_vel_joy")]),
    ])
