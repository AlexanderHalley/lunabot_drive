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
    enable_rgb = LaunchConfiguration('enable_rgb')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')

    # Default RViz config path
    default_rviz_config = PathJoinSubstitution([
        pkg_share, 'config', 'rviz', 'oak_d_camera.rviz'
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
            'enable_rgb',
            default_value='true',
            description='Enable RGB camera stream (reduces latency when disabled)'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable depth stream (reduces latency when disabled)'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='true',
            description='Enable point cloud generation (reduces latency when disabled)'
        ),

        # Include existing camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'oak_d_camera.launch.py'])
            ]),
            launch_arguments={
                'enable_rgb': enable_rgb,
                'enable_depth': enable_depth,
                'enable_pointcloud': enable_pointcloud,
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
                      'oak_link', 'oak_rgb_camera_optical_frame'],
            output='log'
        ),

        # Stereo/depth optical frame (same rotation as RGB)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='oak_stereo_optical_broadcaster',
            arguments=['0', '0', '0', '-1.5707963', '0', '-1.5707963',
                      'oak_link', 'oak_stereo_camera_optical_frame'],
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
