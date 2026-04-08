from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='lunabot_drive',
            executable='mission_state_node',
            name='mission_state_node',
            output='screen',
        ),

        Node(
            package='lunabot_drive',
            executable='bandwidth_monitor_node',
            name='bandwidth_monitor_node',
            output='screen',
        ),

        Node(
            package='lunabot_drive',
            executable='health_monitor_node',
            name='health_monitor_node',
            output='screen',
        ),

        Node(
            package='lunabot_drive',
            executable='wifi_monitor_node',
            name='wifi_monitor_node',
            output='screen',
        ),

        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090}],
            output='screen',
        ),

    ])
