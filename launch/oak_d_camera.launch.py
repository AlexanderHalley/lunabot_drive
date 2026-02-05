from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')

    # Launch argument
    config_file = LaunchConfiguration('config')

    # Default config path
    default_config = os.path.join(pkg_share, 'config', 'oak_d_camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to camera config file'
        ),

        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak',
            output='screen',
            parameters=[config_file]
        ),
    ])
