# Copyright 2027 Lunabot. Licensed under the MIT License.

r"""Convenience wrapper for the ROS side of a simulation run.

    ros2 launch lunabot_bringup sim.launch.py

Equivalent to:

    ros2 launch lunabot_bringup robot.launch.py \\
        hw:=sim use_sim_time:=true camera:=false perception:=true rviz:=true

It starts NO Isaac process. Isaac runs separately, and must be started FIRST:

    src/lunabot_sim/scripts/run_isaac_sim.sh --seed 7

Two processes on purpose -- Isaac's embedded Python is not the ROS distro's,
and the bridge extension links its own DDS. It also means Isaac can be
restarted without tearing down the ROS graph, and vice versa.

The wait-for-clock guard below exists because the failure mode otherwise is
silent: with use_sim_time and no /clock, every node blocks at time zero
without an error or a log line, and the stack simply appears hung.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'slam',
        default_value='rtabmap',
        choices=['none', 'rtabmap', 'cuvslam'],
        description='SLAM backend. rtabmap runs on CPU; cuvslam needs a GPU.',
    ),
    DeclareLaunchArgument(
        'perception',
        default_value='true',
        description="Start boulder detection against Isaac's point cloud.",
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2.',
    ),
    DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='Start joystick teleop and twist_mux.',
    ),
    DeclareLaunchArgument(
        'odom_source',
        default_value='wheel',
        choices=['wheel', 'visual', 'ekf'],
        description='Who publishes odom -> base_link.',
    ),
    DeclareLaunchArgument(
        'wait_for_clock',
        default_value='true',
        description=(
            'Block until Isaac publishes /clock before starting the stack. '
            'Turning this off with Isaac not yet running gives a graph that '
            'hangs at time zero with no error.'
        ),
    ),
]


def generate_launch_description():
    # `ros2 topic echo --once` returns as soon as one message arrives and
    # exits non-zero on timeout, which makes it a serviceable barrier without
    # writing a node for it.
    wait_for_clock = ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'echo',
            '--once',
            '--timeout',
            '120',
            '/clock',
            'rosgraph_msgs/msg/Clock',
        ],
        name='wait_for_clock',
        output='screen',
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lunabot_bringup'), 'launch', 'robot.launch.py']
            )
        ),
        launch_arguments={
            'hw': 'sim',
            'use_sim_time': 'true',
            # Isaac publishes the camera topics itself. Starting the depthai
            # driver too would put two publishers on each -- and it would fail
            # anyway, with no camera plugged in.
            'camera': 'false',
            'slam': LaunchConfiguration('slam'),
            'perception': LaunchConfiguration('perception'),
            'teleop': LaunchConfiguration('teleop'),
            'odom_source': LaunchConfiguration('odom_source'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    return LaunchDescription(
        ARGUMENTS
        + [
            LogInfo(
                msg=(
                    'Waiting for /clock from Isaac Sim. If this hangs, Isaac is not '
                    'running -- start it with '
                    'src/lunabot_sim/scripts/run_isaac_sim.sh'
                )
            ),
            wait_for_clock,
            RegisterEventHandler(OnProcessExit(target_action=wait_for_clock, on_exit=[robot])),
        ]
    )
