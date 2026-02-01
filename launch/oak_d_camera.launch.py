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
            name='oak_d',
            output='screen',
            parameters=[config_file]
        ),

        # Point cloud generation from depth image (on Pi to save network bandwidth)
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='point_cloud_xyz',
            remappings=[
                ('image_rect', '/oak_d/stereo/image_raw'),
                ('camera_info', '/oak_d/stereo/camera_info'),
                ('points', '/oak_d/points'),
            ],
            parameters=[{
                'queue_size': 10,
                'approximate_sync': True,  # Use approximate time sync to avoid sync issues
                'decimation': 4,  # Skip every 4th point in both x and y (16x fewer points)
                'min_range': 0.0,  # Minimum depth in meters
                'max_range': 2.0,  # Maximum depth in meters (filter out points beyond 2m)
            }],
            output='log'
        ),
    ])
