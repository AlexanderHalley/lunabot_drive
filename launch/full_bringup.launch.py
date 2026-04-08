"""
Full system bringup: hardware + bucket actuators.

Includes:
  - hardware_bringup.launch.py (motors, camera, EKF, teleop)
  - bucket_bringup.launch.py   (lift + tilt actuators)

Usage:
    ros2 launch lunabot_drive full_bringup.launch.py
    ros2 launch lunabot_drive full_bringup.launch.py enable_teleop:=false
    ros2 launch lunabot_drive full_bringup.launch.py use_ekf:=false
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_teleop',
            default_value='true',
            description='Enable joystick teleop nodes',
        ),
        DeclareLaunchArgument(
            'use_ekf',
            default_value='true',
            description='Run robot_localization EKF',
        ),

        #Hardware bringup (motors, camera, EKF, teleop)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'hardware_bringup.launch.py'])
            ),
            launch_arguments={
                'enable_teleop': LaunchConfiguration('enable_teleop'),
                'use_ekf': LaunchConfiguration('use_ekf'),
            }.items(),
        ),

        #Bucket actuators (lift + tilt)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_share, 'launch', 'bucket_bringup.launch.py'])
            ),
        ),
    ])
