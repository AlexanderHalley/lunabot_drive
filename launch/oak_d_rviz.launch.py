#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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

    # Process the URDF xacro
    urdf_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

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
            description='Camera config file'
        ),

        # Include existing camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_share, 'launch', 'oak_d_camera.launch.py'
                ])
            ]),
            launch_arguments={
                'config': camera_config,
            }.items(),
            condition=IfCondition(launch_camera)
        ),

        # Robot state publisher (publishes TF from URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
            }],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        # Joint states come from the Pi's drive_node over the network
        # For testing without the robot, uncomment the joint_state_publisher below:
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        # ),

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
