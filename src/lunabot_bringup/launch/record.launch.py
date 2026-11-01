# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Record a run: config/record_topics.yaml, into a new bag per launch.

    ros2 launch lunabot_bringup robot.launch.py ... record:=true

Writes ~/lunabotics/data/bags/run_<date>_<time>/ (MCAP, Jazzy's default,
which Foxglove opens directly -- load config/foxglove/mission_control.json
and the replay looks like the live run). bag_dir:= to put it elsewhere.
"""

import datetime
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Stamp the bag with /clock (sim) instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'bag_dir',
        default_value=os.path.join('~', 'lunabotics', 'data', 'bags'),
        description='Where run_<date>_<time>/ goes (lunabotics-filesystem-setup.md: data/bags).',
    ),
]


def topics(path) -> list[str]:
    """Return the topic list from a record_topics.yaml."""
    with open(path) as f:
        return list(yaml.safe_load(f)['topics'])


def _record(context):
    share = FindPackageShare('lunabot_bringup').perform(context)
    bag_dir = os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))
    os.makedirs(bag_dir, exist_ok=True)
    name = datetime.datetime.now().strftime('run_%Y-%m-%d_%H%M%S')
    # rosbag2's recorder NODE, not `ros2 bag record`: the CLI wrapper ignored
    # launch's SIGINT once in Isaac (the whole launch then hung on shutdown,
    # bag unfinished, until a manual SIGTERM); the node shuts down on SIGINT
    # like every other node here.
    return [
        Node(
            package='rosbag2_transport',
            executable='recorder',
            name='rosbag2_recorder',
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'record.topics': topics(os.path.join(share, 'config', 'record_topics.yaml')),
                    'storage.uri': os.path.join(bag_dir, name),
                }
            ],
            output='log',
        )
    ]


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_record)])
