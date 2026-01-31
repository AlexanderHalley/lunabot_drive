from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')
    config_file = os.path.join(pkg_share, 'config', 'oak_d_camera.yaml')

    # Launch configurations
    enable_rgb = LaunchConfiguration('enable_rgb')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_rgb',
            default_value='true',
            description='Enable RGB camera stream'
        ),
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
            executable='camera_node',
            name='oak_d',
            output='screen',
            parameters=[
                config_file,
                {
                    'rgb.i_publish_topic': enable_rgb,
                    'stereo.i_publish_topic': enable_depth,
                    'pointcloud.i_enable': enable_pointcloud,
                }
            ]
        ),
    ])
