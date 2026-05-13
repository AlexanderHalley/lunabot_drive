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

        # Hardcoded optical-frame TFs.
        # Args: x y z yaw pitch roll parent child. The optical convention
        # (Z-fwd, X-right, Y-down) is reached from REP-103 (X-fwd, Y-left, Z-up)
        # via yaw=-pi/2, pitch=0, roll=-pi/2. Translation offsets (~few cm baseline)
        # are ignored — acceptable for arena-scale SLAM / AprilTag.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='oak_rgb_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'oak_mount', 'oak_rgb_camera_optical_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='oak_right_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'oak_mount', 'oak_right_camera_optical_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='oak_left_optical_tf',
            arguments=['0', '0', '0', '-1.5708', '0', '-1.5708',
                       'oak_mount', 'oak_left_camera_optical_frame'],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='oak_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0',
                       'oak_mount', 'oak_imu_frame'],
        ),
    ])
