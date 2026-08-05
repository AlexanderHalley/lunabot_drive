"""Boulder detection, and depth normalisation on real hardware.

The detector consumes /oak_d/points and publishes /perception/boulders in
base_link. Identical in sim and on the robot.

depth_normalizer runs on REAL HARDWARE ONLY: the OAK-D publishes depth as
16UC1 millimetres while Isaac publishes 32FC1 metres, and normalising in sim
would be converting data that is already correct.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_perception'), 'config', 'boulder_detector.yaml']
        ),
        description='Boulder detector parameters.',
    ),
    DeclareLaunchArgument(
        'cloud_topic',
        default_value='/oak_d/points',
        description='Input point cloud. Metres in both sim and hardware.',
    ),
    DeclareLaunchArgument(
        'normalize_depth',
        default_value='false',
        description=(
            'Convert 16UC1 millimetre depth to 32FC1 metres. True on real '
            'hardware, false in sim where depth is already 32FC1 metres.'
        ),
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    detector = Node(
        package='lunabot_perception',
        executable='boulder_detector',
        name='boulder_detector',
        parameters=[LaunchConfiguration('config'), {'use_sim_time': use_sim_time}],
        remappings=[('/oak_d/points', LaunchConfiguration('cloud_topic'))],
        output='screen',
    )

    depth_normalizer = Node(
        package='lunabot_perception',
        executable='depth_normalizer',
        name='depth_normalizer',
        condition=IfCondition(LaunchConfiguration('normalize_depth')),
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('depth_in', '/oak_d/stereo/image_raw'),
            ('depth_out', '/oak_d/stereo/image_metres'),
        ],
        output='log',
    )

    return LaunchDescription(ARGUMENTS + [detector, depth_normalizer])
