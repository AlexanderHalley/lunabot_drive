# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""/diagnostics for the running stack, plus the aggregator that groups it.

Two nodes, and they are for different audiences:

    robot_health        publishes /diagnostics. Foxglove's Diagnostics panel
                        and PlotJuggler read this directly.
    aggregator_node     publishes /diagnostics_agg. rqt_robot_monitor reads
                        ONLY this, and shows an empty tree without it.

Started by robot.launch.py under diagnostics:=true, which is the default --
a health topic nobody launches is the same as no health topic at all. It
costs two small nodes and subscribes to nothing heavy; see the module
docstring in scripts/robot_health.py for why that last part took care.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        'profile',
        default_value='mock',
        choices=['mock', 'sim', 'real'],
        description=(
            'Which topics to expect, matching the hw argument. sim adds /clock; '
            'real adds /drive/status and the drivetrain report.'
        ),
    ),
    DeclareLaunchArgument(
        'camera',
        default_value='false',
        description=(
            'Watch the camera. Monitored through camera_info rather than the '
            'image topics, which are far too expensive to subscribe to for a '
            'liveness check.'
        ),
    ),
    DeclareLaunchArgument(
        'perception',
        default_value='false',
        description='Watch /perception/boulders, which also stands in for the point cloud.',
    ),
    DeclareLaunchArgument(
        'aggregator',
        default_value='true',
        description=(
            'Also run diagnostic_aggregator, which publishes /diagnostics_agg. '
            'Needed by rqt_robot_monitor and by nothing else.'
        ),
    ),
    DeclareLaunchArgument(
        'diagnostics_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_bringup'), 'config', 'diagnostics.yaml']
        ),
        description='Analyzer configuration for the aggregator.',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    health = Node(
        package='lunabot_bringup',
        executable='robot_health.py',
        name='robot_health',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'profile': LaunchConfiguration('profile'),
                'camera': LaunchConfiguration('camera'),
                'perception': LaunchConfiguration('perception'),
            }
        ],
        output='log',
    )

    aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        condition=IfCondition(LaunchConfiguration('aggregator')),
        parameters=[
            LaunchConfiguration('diagnostics_config'),
            {'use_sim_time': use_sim_time},
        ],
        output='log',
    )

    return LaunchDescription(ARGUMENTS + [health, aggregator])
