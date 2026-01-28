from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lunabot_drive',
            executable='drive_node',
            parameters=[{
                'can_interface': 'can0',
                'left_front_id': 1,
                'right_front_id': 2,
                'left_rear_id': 3,
                'right_rear_id': 4,
                'wheel_base': 0.5,
                'max_duty_cycle': 0.8
            }]
        ),
    ])
