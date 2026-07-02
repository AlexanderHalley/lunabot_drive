"""ROS-side bring-up for Isaac Sim (assumes the Isaac stage is already running
with the ROS 2 Bridge action graph active).

    ros2 launch lunabot_simulation isaac_sim.launch.py drive_type:=diff

Starts robot_state_publisher (sim_mode=isaac), the drivetrain controllers, and
twist_mux — the same nodes used on hardware, so autonomy developed here transfers
unchanged. Note: Isaac Sim itself is launched separately (GUI or
isaacsim python app); this only wires the ROS graph. All nodes use sim time.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_type = LaunchConfiguration("drive_type")
    description = FindPackageShare("lunabot_description")
    control = FindPackageShare("lunabot_control")

    return LaunchDescription([
        DeclareLaunchArgument("drive_type", default_value="diff",
                              description="diff | skid | mecanum"),
        # use_sim_time everywhere when driven by Isaac /clock
        SetLaunchConfiguration("use_sim_time", "true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [description, "launch", "description.launch.py"])),
            launch_arguments={"drive_type": drive_type, "sim_mode": "isaac"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [control, "launch", "control.launch.py"])),
            launch_arguments={"drive_type": drive_type}.items(),
        ),
    ])
