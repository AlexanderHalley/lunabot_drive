"""Full stack on the real robot: description(sim_mode=none) + real
controller_manager (lunabot_hardware SparkFlex plugin) + control + perception +
navigation.

    ros2 launch lunabot_bringup robot.launch.py drive_type:=skid

STRUCTURAL STUB — depends on lunabot_hardware being implemented (Phase 3). Until
then the 2026 lunabot_drive package still drives the real robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drive_type = LaunchConfiguration("drive_type")
    description = FindPackageShare("lunabot_description")
    control = FindPackageShare("lunabot_control")

    xacro_file = PathJoinSubstitution([description, "urdf", "lunabot.urdf.xacro"])
    robot_description = Command(
        ["xacro ", xacro_file, " drive_type:=", drive_type, " sim_mode:=none"])
    controllers = PathJoinSubstitution(
        [control, "config", "controllers_diff_drive.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("drive_type", default_value="diff"),

        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),

        # Real controller_manager loads the lunabot_hardware SparkFlex plugin
        # (declared in the <ros2_control> URDF tag) + controller configs.
        Node(package="controller_manager", executable="ros2_control_node",
             parameters=[{"robot_description": robot_description}, controllers],
             output="screen"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [control, "launch", "control.launch.py"])),
            launch_arguments={"drive_type": drive_type}.items(),
        ),
        # TODO: include perception (use_camera:=true) + navigation (use_sim_time:=false).
    ])
