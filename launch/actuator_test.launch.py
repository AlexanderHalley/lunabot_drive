"""
Single-actuator test launch file.

Usage:
  ros2 launch lunabot_drive actuator_test.launch.py which:=lift
  ros2 launch lunabot_drive actuator_test.launch.py which:=tilt
  ros2 launch lunabot_drive actuator_test.launch.py which:=lift home_on_startup:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Per-actuator defaults — mirrors config/bucket_actuators.yaml
_DEFAULTS = {
    "lift": {
        "actuator_name": "lift",
        "rpwm_gpio": 12,
        "lpwm_gpio": 18,
        "en_gpio": 5,
        "hall_a_gpio": 22,
        "hall_b_gpio": 23,
        "stroke_mm": 203.2,
        "joint_name": "bucket_lift_joint",
        "position_to_joint_scale": 0.001,
    },
    "tilt": {
        "actuator_name": "tilt",
        "rpwm_gpio": 13,
        "lpwm_gpio": 19,
        "en_gpio": 6,
        "hall_a_gpio": 24,
        "hall_b_gpio": 25,
        "stroke_mm": 254.0,
        "joint_name": "bucket_tilt_joint",
        "position_to_joint_scale": 1.0,
    },
}


def _launch_setup(context):
    which = LaunchConfiguration("which").perform(context)
    home_on_startup = LaunchConfiguration("home_on_startup").perform(context)

    if which not in _DEFAULTS:
        raise ValueError(f"which:={which} is invalid — must be 'lift' or 'tilt'")

    params = dict(_DEFAULTS[which])
    params["home_on_startup"] = home_on_startup.lower() == "true"

    node = Node(
        package="lunabot_drive",
        executable="actuator_driver_node",
        name="actuator_driver",
        namespace=f"bucket/{which}",
        parameters=[params],
        output="screen",
    )
    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "which",
            default_value="lift",
            description="Which actuator to test: 'lift' or 'tilt'",
        ),
        DeclareLaunchArgument(
            "home_on_startup",
            default_value="true",
            description="Run homing sequence on startup",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
