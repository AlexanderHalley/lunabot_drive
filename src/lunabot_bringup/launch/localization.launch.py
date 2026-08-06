# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""robot_localization EKF, for odom_source:=ekf.

Fuses wheel odometry with the IMU into odom -> base_link. Started only when
the EKF is the odom source; with odom_source:=wheel or :=visual this file
runs nothing, because two publishers on odom -> base_link gives a TF tree
that looks correct and behaves nondeterministically.

See config/ekf.yaml for what it can and cannot fix with no encoders.
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
        'ekf_config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_bringup'), 'config', 'ekf.yaml']
        ),
        description='robot_localization parameter file.',
    ),
]


def generate_launch_description():
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        parameters=[
            LaunchConfiguration('ekf_config'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # The EKF's own output. Deliberately NOT /odom: that is the wheel
        # odometry it consumes, and remapping its output on top of its input
        # is a feedback loop.
        remappings=[('odometry/filtered', '/odometry/filtered')],
        output='screen',
    )

    return LaunchDescription(ARGUMENTS + [ekf])
