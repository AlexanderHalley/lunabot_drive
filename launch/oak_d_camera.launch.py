from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')
    config_file = os.path.join(pkg_share, 'config', 'oak_d_camera.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable depth stream'
        ),
        DeclareLaunchArgument(
            'enable_pointcloud',
            default_value='true',
            description='Enable point cloud generation'
        ),

        Node(
            package='depthai_ros_driver',
            executable='camera',
            name='oak_d',
            output='screen',
            parameters=[config_file],
            remappings=[
                # Remap to standard /camera/* namespace
                ('/oak/rgb/image_raw', '/camera/image_raw'),
                ('/oak/rgb/camera_info', '/camera/camera_info'),
                ('/oak/stereo/depth', '/camera/depth/image_raw'),
                ('/oak/stereo/camera_info', '/camera/depth/camera_info'),
                ('/oak/points', '/camera/depth/points'),
            ]
        ),
    ])
