"""Publish the robot description and its TF tree.

Runs `robot_state_publisher`, which expands the xacro, publishes it on
/robot_description (transient local, so late subscribers still get it), and
turns /joint_states into the base_link -> * transforms.

This is identical for mock, sim and real hardware. In sim it is what makes
Isaac's ROS2PublishTransformTree unnecessary: TF comes from the same URDF and
the same node in both cases, so the tree cannot diverge between them.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument(
        'hardware',
        default_value='mock',
        choices=['mock', 'sim', 'real'],
        description='Which ros2_control hardware plugin the description selects.',
    ),
    DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='SocketCAN device for hardware:=real. Use vcan0 to test without motors.',
    ),
    DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Frame and joint name prefix.',
    ),
    DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Emit the <ros2_control> block. False gives geometry and TF only.',
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
]


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('lunabot_description'), 'urdf', 'lunabot.urdf.xacro']
    )

    # ParameterValue(..., value_type=str) is not optional. Without it, a
    # description whose first character is a digit or whose content looks
    # like YAML gets coerced to the wrong type and robot_state_publisher
    # fails with a message that does not mention the real cause.
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_file,
                ' hardware:=',
                LaunchConfiguration('hardware'),
                ' can_interface:=',
                LaunchConfiguration('can_interface'),
                ' prefix:=',
                LaunchConfiguration('prefix'),
                ' use_ros2_control:=',
                LaunchConfiguration('use_ros2_control'),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return LaunchDescription(ARGUMENTS + [robot_state_publisher])
