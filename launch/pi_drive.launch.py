from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Wheel parameters from URDF (description/robot_core.xacro)
    wheel_radius = 0.1778        # 7 inches
    wheel_base = 0.762           # distance between left and right wheels (2 * wheel_offset_y)

    return LaunchDescription([
        Node(
            package='lunabot_drive',
            executable='drive_node',
            parameters=[{
                'can_interface': 'can0',
                'left_front_id': 2,
                'right_front_id': 1,
                'left_rear_id': 3,
                'right_rear_id': 4,
                'wheel_base': wheel_base,
                'wheel_radius': wheel_radius,
                'gear_ratio': 1.0,          # Adjust if using gearbox
                'max_duty_cycle': 0.8,
                'joint_state_rate': 50.0,   # Hz - rate to publish joint states
            }]
        ),
    ])
