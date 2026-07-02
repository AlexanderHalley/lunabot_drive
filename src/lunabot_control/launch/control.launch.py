"""Spawn the controller_manager controllers for the selected drivetrain.

    ros2 launch lunabot_control control.launch.py drive_type:=diff

Picks the matching controller YAML, spawns joint_state_broadcaster + the drive
controller, and starts twist_mux. Assumes the controller_manager is already
running (brought up by lunabot_bringup with the robot_description that contains
the <ros2_control> tag), or launch it here in a real bring-up.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("lunabot_control")
    drive_type = LaunchConfiguration("drive_type")

    # diff/skid share the diff_drive config; mecanum uses its own.
    controller_name = PythonExpression([
        "'mecanum_drive_controller' if '", drive_type,
        "' == 'mecanum' else 'diff_drive_controller'"
    ])
    controllers_yaml = PathJoinSubstitution([
        pkg, "config",
        PythonExpression([
            "'controllers_mecanum.yaml' if '", drive_type,
            "' == 'mecanum' else 'controllers_diff_drive.yaml'"
        ]),
    ])
    twist_mux_yaml = PathJoinSubstitution([pkg, "config", "twist_mux.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("drive_type", default_value="diff",
                              description="diff | skid | mecanum"),

        Node(package="controller_manager", executable="spawner",
             arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner",
             arguments=[controller_name, "--param-file", controllers_yaml]),

        Node(package="twist_mux", executable="twist_mux",
             parameters=[twist_mux_yaml],
             remappings=[("cmd_vel_out", "cmd_vel")]),
    ])
