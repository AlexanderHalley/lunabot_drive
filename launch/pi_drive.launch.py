import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('lunabot_drive')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_file = os.path.join(pkg_share, 'urdf', 'lunabot.urdf.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
        }],
    )

    drive_node = Node(
        package='lunabot_drive',
        executable='drive_node',
        parameters=[{
            'can_interface': 'can0',
            'left_front_id': 2,
            'right_front_id': 1,
            'left_rear_id': 3,
            'right_rear_id': 4,
            'wheel_base': 0.5,
            'max_duty_cycle': 0.8,
            'wheel_radius': 0.075,
            'encoder_cpr': 42,
            'gear_ratio': 100.0,
            'odom_rate': 30.0,
            'odom_frame_id': 'odom',
            'odom_child_frame_id': 'base_link',
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        drive_node,
        ekf_node,
    ])
