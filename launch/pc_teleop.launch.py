from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'device_id': 0}]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{
                'require_enable_button': False,                
                # Axis mappings for Switch Pro Controller
                'axis_linear.x': 1,        # Left stick vertical
                'axis_linear.y': -1,
                'axis_linear.z': -1,
                'axis_angular.yaw': 0,     # Left stick horizontal
                'axis_angular.pitch': -1,
                'axis_angular.roll': -1,
                
                # Speed scales
                'scale_linear.x': 0.5,     # Normal speed
                'scale_linear.y': 0.0,
                'scale_linear.z': 0.0,
                'scale_angular.yaw': 0.5,
                'scale_angular.pitch': 0.0,
                'scale_angular.roll': 0.0,
                
            }]
        ),
    ])
