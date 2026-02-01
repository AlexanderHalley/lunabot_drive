#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('lunabot_drive')

    # Launch arguments
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    launch_camera = LaunchConfiguration('launch_camera')
    camera_config = LaunchConfiguration('camera_config')

    # Default paths
    default_rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'rviz', 'oak_d_camera.rviz'
    ])
    default_camera_config = PathJoinSubstitution([
        pkg_share, 'config', 'oak_d_camera.yaml'
    ])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            'launch_camera',
            default_value='true',
            description='Launch Oak-D camera node'
        ),
        DeclareLaunchArgument(
            'camera_config',
            default_value=default_camera_config,
            description='Camera config file (oak_d_camera.yaml, oak_d_rgb_only.yaml, or oak_d_pointcloud_only.yaml)'
        ),

        # Include existing camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            launch_arguments={
                'config': camera_config,
            }.items(),
            condition=IfCondition(launch_camera)
        ),

        # Static TF publishers for camera frames
        # Base frame: world -> oak_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_base_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'oak_link'],
            output='log'
        ),

        # RGB optical frame (optical convention: X=right, Y=down, Z=forward)
        # Rotation from standard frame: -90° pitch, -90° yaw
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_rgb_optical_broadcaster',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                      'oak_link', 'oak_d_rgb_camera_optical_frame'],
            output='log'
        ),

        # Stereo/depth optical frame (same rotation as RGB)
        # Note: In RGBD mode, stereo uses RGB frame, so we make them identical
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_stereo_optical_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0',
                      'oak_d_rgb_camera_optical_frame', 'oak_stereo_camera_optical_frame'],
            output='log'
        ),

        # IMU frame (coincident with oak_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_imu_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0',
                      'oak_link', 'oak_imu_frame'],
            output='log'
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
