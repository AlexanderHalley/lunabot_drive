# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""SLAM, with a swappable backend.

    ros2 launch lunabot_slam slam.launch.py backend:=rtabmap
    ros2 launch lunabot_slam slam.launch.py backend:=cuvslam

Every topic name is declared HERE, once, with defaults taken from
docs/TOPIC_FRAME_CONTRACT.md, and passed down to whichever backend file is
included. Renaming a camera topic is then a one-line edit in one file.

That is not gratuitous indirection -- it is the specific failure mode the 2026
codebase hit, where the README, the launch files and the RViz config each
carried their own copy of the topic names and two of the three were wrong.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'backend',
        default_value='rtabmap',
        choices=['rtabmap', 'cuvslam', 'none'],
        description=(
            'rtabmap runs on CPU anywhere and is the default. '
            'cuvslam needs an NVIDIA GPU and an Isaac ROS release for this distro; '
            'it is scaffolding until that is confirmed (see docs/SLAM.md). '
            'none is for driving without mapping.'
        ),
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        description=(
            'Localise against an existing map instead of building one. '
            'Mapping is destructive: the default rebuilds from scratch each run.'
        ),
    ),
    DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description=(
            'Whether the backend publishes odom -> base_link. False means '
            'diff_drive_controller owns it (odom_source:=wheel). Exactly one '
            'node may publish this transform.'
        ),
    ),
    # ---- Topics. Defaults from docs/TOPIC_FRAME_CONTRACT.md. ----
    DeclareLaunchArgument(
        'rgb_topic',
        default_value='/oak_d/rgb/image_rect',
        description='Rectified RGB image. rtabmap only.',
    ),
    DeclareLaunchArgument(
        'depth_topic',
        default_value='/oak_d/stereo/image_raw',
        description='Depth image, registered to the RGB frame. rtabmap only.',
    ),
    DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/oak_d/rgb/camera_info',
        description='Intrinsics for rgb_topic. rtabmap only.',
    ),
    DeclareLaunchArgument(
        'left_rect_topic',
        default_value='/oak_d/left/image_rect',
        description='Rectified left mono image. cuVSLAM only.',
    ),
    DeclareLaunchArgument(
        'right_rect_topic',
        default_value='/oak_d/right/image_rect',
        description='Rectified right mono image. cuVSLAM only.',
    ),
    DeclareLaunchArgument(
        'left_info_topic',
        default_value='/oak_d/left/camera_info',
        description='Intrinsics for left_rect_topic. cuVSLAM only.',
    ),
    DeclareLaunchArgument(
        'right_info_topic',
        default_value='/oak_d/right/camera_info',
        description='Intrinsics for right_rect_topic. cuVSLAM only.',
    ),
    DeclareLaunchArgument(
        'imu_topic',
        default_value='/oak_d/imu/data',
        description='IMU. Used by both backends for gravity and rotation constraints.',
    ),
    DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Wheel odometry, as a motion prior.',
    ),
]

# Which of the topic arguments each backend actually consumes. Passing a
# cuVSLAM topic to rtabmap would be silently ignored, which is the kind of
# thing that makes a launch file impossible to reason about.
_SHARED = ('use_sim_time', 'localization', 'publish_odom_tf', 'imu_topic', 'odom_topic')
_RTABMAP_ONLY = ('rgb_topic', 'depth_topic', 'camera_info_topic')
_CUVSLAM_ONLY = (
    'left_rect_topic',
    'right_rect_topic',
    'left_info_topic',
    'right_info_topic',
)


def generate_launch_description():
    pkg = FindPackageShare('lunabot_slam')

    def backend(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', f'{name}.launch.py'])
            ),
            condition=LaunchConfigurationEquals('backend', name),
            launch_arguments={
                key: LaunchConfiguration(key) for key in _SHARED + arguments
            }.items(),
        )

    return LaunchDescription(
        ARGUMENTS
        + [
            backend('rtabmap', _RTABMAP_ONLY),
            backend('cuvslam', _CUVSLAM_ONLY),
        ]
    )
