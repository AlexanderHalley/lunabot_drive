"""
Single-actuator test launch file.

Usage:
  ros2 launch lunabot_drive actuator_test.launch.py which:=lift
  ros2 launch lunabot_drive actuator_test.launch.py which:=tilt

Command the actuator (keep publishing to hold motor on):
  ros2 topic pub /bucket/lift/actuator_driver/command std_msgs/msg/Float64 "data: 1.0" --rate 5
  ros2 topic pub /bucket/lift/actuator_driver/command std_msgs/msg/Float64 "data: -1.0" --rate 5
  ros2 topic pub /bucket/lift/actuator_driver/command std_msgs/msg/Float64 "data: 0.0" --rate 5
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_DEFAULTS = {
    "lift": {
        "actuator_name": "lift",
        "rpwm_gpio": 12,
        "lpwm_gpio": 18,
        "en_gpio": 5,
    },
    "tilt": {
        "actuator_name": "tilt",
        "rpwm_gpio": 13,
        "lpwm_gpio": 19,
        "en_gpio": 6,
    },
}


def _launch_setup(context):
    which = LaunchConfiguration("which").perform(context)
    if which not in _DEFAULTS:
        raise ValueError(f"which:={which} is invalid — must be 'lift' or 'tilt'")

    node = Node(
        package="lunabot_drive",
        executable="actuator_driver_node",
        name="actuator_driver",
        namespace=f"bucket/{which}",
        parameters=[_DEFAULTS[which]],
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
        OpaqueFunction(function=_launch_setup),
    ])
