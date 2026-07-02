"""Publish the Lunabot description (robot_state_publisher) with a selectable
drivetrain. Other stacks include this; run standalone to inspect TF in RViz.

    ros2 launch lunabot_description description.launch.py drive_type:=diff gui:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lunabot_description")
    drive_type = LaunchConfiguration("drive_type")
    sim_mode = LaunchConfiguration("sim_mode")
    gui = LaunchConfiguration("gui")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "lunabot.urdf.xacro"])
    robot_description = Command(
        ["xacro ", xacro_file, " drive_type:=", drive_type, " sim_mode:=", sim_mode]
    )

    return LaunchDescription([
        DeclareLaunchArgument("drive_type", default_value="diff",
                              description="diff | skid | mecanum"),
        DeclareLaunchArgument("sim_mode", default_value="none",
                              description="none | isaac | mock"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="launch joint_state_publisher_gui"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        ),
    ])
