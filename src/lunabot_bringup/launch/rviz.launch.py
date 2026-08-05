"""RViz2 with one of the configs from lunabot_description.

Split into its own file for one reason: use_sim_time. RViz is the node people
forget to pass it to, and the symptom is not an error -- it is TF displays
that freeze or lag while everything else looks fine, which reads as a TF bug
and is not one.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'rviz_config',
        default_value='slam',
        choices=['description', 'slam'],
        description=(
            'description = model and TF only, fixed frame base_link. '
            'slam = map, cloud and detections, fixed frame map (shows nothing '
            'until a SLAM backend is publishing map -> odom).'
        ),
    ),
]


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare('lunabot_description'),
            'rviz',
            [LaunchConfiguration('rviz_config'), '.rviz'],
        ]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='log',
    )

    return LaunchDescription(ARGUMENTS + [rviz])
