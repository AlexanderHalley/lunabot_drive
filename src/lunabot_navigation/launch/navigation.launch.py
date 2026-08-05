"""Nav2, configured for the Lunabot rover.

SKELETON. Not part of the working path yet -- see the header of
config/nav2_params.yaml. It exists so the frame names, footprint and velocity
limits agree with the rest of the workspace from the start.

Nav2 output goes to /cmd_vel_nav, not /cmd_vel. twist_mux arbitrates, and
teleop outranks navigation: a human reaching for the controller is a human who
wants the rover to stop doing what it is doing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_navigation'), 'config', 'nav2_params.yaml']
        ),
        description='Nav2 parameter file.',
    ),
    DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Bring the Nav2 lifecycle nodes up automatically.',
    ),
    DeclareLaunchArgument(
        'use_composition',
        default_value='true',
        description=(
            'Load Nav2 nodes into a single container. Worth keeping true on '
            'an embedded board -- it avoids serialising costmaps between '
            'processes.'
        ),
    ),
]


def generate_launch_description():
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_composition': LaunchConfiguration('use_composition'),
            # Deliberately NOT nav2_bringup's full bringup_launch.py: that
            # also starts AMCL and map_server, and rtabmap already owns
            # map -> odom and publishes /map. Two publishers on map -> odom
            # gives a robot that teleports between two beliefs about where it
            # is.
        }.items(),
    )

    return LaunchDescription(ARGUMENTS + [nav2])
