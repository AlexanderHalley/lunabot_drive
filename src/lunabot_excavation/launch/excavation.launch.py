# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The excavation server, included by robot.launch.py (excavation:=true).

ros2 launch lunabot_excavation excavation.launch.py excavator_backend:=model

NOT `backend`: launch arguments are shared down every include, and
slam.launch.py's `backend:=rtabmap` reached this file and failed its choices,
which took the whole stack down (Isaac, 2026-11-10).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'excavator_backend',
        default_value='model',
        choices=['model'],
        description='model: a kinematic stand-in that drives nothing (no drivers yet).',
    ),
    DeclareLaunchArgument('use_sim_time', default_value='false'),
]


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare('lunabot_excavation'), 'config', 'excavator.yaml']
    )
    excavator = Node(
        package='lunabot_excavation',
        executable='excavator.py',
        name='excavator',
        output='screen',
        # The params file first: whichever comes later wins.
        parameters=[
            config,
            {
                'backend': LaunchConfiguration('excavator_backend'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )
    return LaunchDescription(ARGUMENTS + [excavator])
