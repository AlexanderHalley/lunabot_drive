# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

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
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
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
    DeclareLaunchArgument(
        'clock_timeout',
        default_value='120',
        description=(
            'Seconds to wait for /clock before giving up. A first Isaac run on '
            'a new machine compiles shaders and can take considerably longer '
            'than a warm one; raise this rather than switching the wait off.'
        ),
    ),
]


def _robot_stack(condition=None):
    """Build the stack itself, under hw:=sim.

    A factory rather than a variable because launch actions are stateful and
    cannot appear twice in one description: this is instantiated once for the
    wait_for_clock:=false path and once from the barrier's exit handler.
    """
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('lunabot_bringup'), 'launch', 'robot.launch.py']
            )
        ),
        condition=condition,
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


def generate_launch_description():
    wait = LaunchConfiguration('wait_for_clock')

    # `ros2 topic echo --once` returns as soon as one message arrives, which is
    # the barrier. Its own --timeout is NOT used: on expiry ros2cli resolves
    # the wait and returns success, so a timed-out barrier is indistinguishable
    # from a satisfied one and the stack launches into exactly the hang this
    # guard exists to prevent. coreutils `timeout` exits 124 instead, which the
    # handler below can actually act on.
    clock_barrier = ExecuteProcess(
        cmd=[
            'timeout',
            LaunchConfiguration('clock_timeout'),
            'ros2',
            'topic',
            'echo',
            '--once',
            '/clock',
            'rosgraph_msgs/msg/Clock',
        ],
        name='wait_for_clock',
        output='screen',
        condition=IfCondition(wait),
    )

    def on_barrier_exit(event, context):
        if event.returncode == 0:
            return [_robot_stack()]
        # Refusing to start is the point. Bringing the stack up without /clock
        # produces nodes blocked at time zero -- no error, no log line, a graph
        # that merely appears hung -- and diagnosing that costs far more than
        # this message.
        return [
            LogInfo(
                msg=(
                    'No /clock within the timeout, so Isaac Sim is not running or is '
                    'not publishing it. NOT starting the stack: every node would '
                    'block at time zero with no error. Start Isaac first with '
                    'src/lunabot_sim/scripts/run_isaac_sim.sh, or raise '
                    'clock_timeout: on a cold machine the first run compiles '
                    'shaders and can take several minutes.'
                )
            ),
            Shutdown(reason='Isaac Sim is not publishing /clock'),
        ]

    return LaunchDescription(
        ARGUMENTS
        + [
            LogInfo(
                msg=(
                    'Waiting for /clock from Isaac Sim. If this hangs, Isaac is not '
                    'running -- start it with '
                    'src/lunabot_sim/scripts/run_isaac_sim.sh'
                ),
                condition=IfCondition(wait),
            ),
            clock_barrier,
            RegisterEventHandler(
                OnProcessExit(target_action=clock_barrier, on_exit=on_barrier_exit),
                condition=IfCondition(wait),
            ),
            # wait_for_clock:=false used to be declared and then ignored -- the
            # barrier ran unconditionally and the argument did nothing. This is
            # the path that makes it mean something.
            _robot_stack(condition=UnlessCondition(wait)),
        ]
    )
