"""Full stack in Isaac Sim: description(isaac) + control + perception(no camera)
+ navigation, all on sim time.

    ros2 launch lunabot_bringup sim.launch.py drive_type:=diff

STRUCTURAL STUB — wires the includes; enable navigation once it is tuned.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_type = LaunchConfiguration("drive_type")
    return LaunchDescription([
        DeclareLaunchArgument("drive_type", default_value="diff"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("lunabot_simulation"), "launch", "isaac_sim.launch.py"])),
            launch_arguments={"drive_type": drive_type}.items(),
        ),
        # TODO: include lunabot_perception (use_camera:=false) and
        #       lunabot_navigation once Phase 2 begins.
    ])
