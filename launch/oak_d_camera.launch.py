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

        # OAK-D camera driver — publishes /oak/stereo/image_raw (depth) + /oak/imu/data
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak',
            output='screen',
            parameters=[config_file]
        ),

        # Convert depth image → pointcloud.
        # depthai publishes depth with SENSOR_DATA (best_effort) QoS, so we must
        # override the subscription QoS here to match.
        Node(
            package='depth_image_proc',
            executable='point_cloud_xyz_node',
            name='point_cloud_xyz',
            remappings=[
                ('image_rect', '/oak/stereo/image_raw'),
                ('camera_info', '/oak/stereo/camera_info'),
                ('points', '/oak/stereo/points'),
            ],
            parameters=[{
                'qos_overrides./oak/stereo/image_raw.subscription.reliability': 'best_effort',
                'qos_overrides./oak/stereo/image_raw.subscription.durability': 'volatile',
                'qos_overrides./oak/stereo/camera_info.subscription.reliability': 'best_effort',
                'qos_overrides./oak/stereo/camera_info.subscription.durability': 'volatile',
            }],
            output='screen',
        ),
    ])
