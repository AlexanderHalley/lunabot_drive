# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""foxglove_bridge, so Foxglove Studio can attach to the running stack.

    ros2 launch lunabot_bringup robot.launch.py hw:=sim foxglove:=true

Then open Foxglove and connect to ws://<robot>:8765, and import the mission
control layout, config/foxglove/mission_control.json (docs/MONITORING.md
"Mission control"). Over the competition WiFi add foxglove_profile:=field.

The bridge is Apache-2.0 and the Studio app has a free tier; students and
researchers get the paid tier free on a .edu or .ac address, which is what
makes this worth wiring in rather than leaving as a README suggestion. Lichtblick
(the open-source fork) also connects: bridge 3.x speaks only the
foxglove.sdk.v1 WebSocket subprotocol, which both offer.

Why this and not rosbridge: foxglove_bridge reads each publisher's QoS off
the graph and matches it per topic. rosbridge subscribes RELIABLE by default,
which silently never matches this stack's BEST_EFFORT sensor publishers --
/oak_d/points, the image topics, everything on rclcpp::SensorDataQoS. The
symptom is a panel that stays empty with no error, which is the same failure
this workspace pinned QoS in the RViz configs to avoid.

What Foxglove can and cannot draw here:

    3D panel        /robot_description is TRANSIENT_LOCAL, so the URDF loads.
                    TF, point clouds, images and MarkerArrays all work.
    Plot panel      any numeric field of any message, including
                    lunabot_msgs/DriveStatus -- the bridge carries the schema
                    off the type support, so custom messages need no setup.
    Diagnostics     /diagnostics, which diagnostics.launch.py publishes.
    Detections      NOT vision_msgs/Detection3DArray, which Foxglove has no
                    renderer for. Use /perception/debug/markers, which exists
                    for exactly this reason -- see TOPIC_FRAME_CONTRACT.md.

OFF BY DEFAULT because it opens a listening socket. On the competition field
that is a port on a robot; the argument for making it opt-in is the same one
that applies to any network service on a machine you cannot re-image quickly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        'port',
        default_value='8765',
        description='WebSocket port. Foxglove Studio defaults to this one.',
    ),
    DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description=(
            'Bind address. 0.0.0.0 accepts connections from the offboard '
            'laptop; 127.0.0.1 restricts to the robot itself.'
        ),
    ),
    DeclareLaunchArgument(
        'profile',
        default_value='bench',
        choices=['bench', 'field'],
        description=(
            'bench: every topic, service and parameter, for a wired LAN. field: only '
            'what the mission control layout draws, and only its buttons back -- for '
            'the competition WiFi. config/foxglove/<profile>.yaml.'
        ),
    ),
    DeclareLaunchArgument(
        'operator_camera',
        default_value='true',
        description=(
            'Republish the onboard camera small and slow as /operator/camera/compressed '
            '(scripts/operator_camera.py). Lazy: it costs nothing until a panel watches it.'
        ),
    ),
]


def generate_launch_description():
    bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare('lunabot_bringup'),
                    'config',
                    'foxglove',
                    [LaunchConfiguration('profile'), '.yaml'],
                ]
            ),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                # Compress the WebSocket stream. The link to the rover is the
                # bottleneck, not its CPU -- the same reasoning that set the
                # camera to 400P in oak_d_s2.yaml.
                'use_compression': True,
                # 10 MB of outgoing buffer before the bridge starts dropping
                # messages for a slow client. Large enough to ride out a WiFi
                # stall, small enough that a laptop that has wandered out of
                # range does not make the bridge grow without bound on a
                # machine with 8 GB.
                'send_buffer_limit': 10000000,
            },
        ],
        output='log',
    )

    camera = Node(
        package='lunabot_bringup',
        executable='operator_camera.py',
        name='operator_camera',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('operator_camera')),
        output='log',
    )

    # The layout's E-STOP / Release buttons call its /e_stop/set service.
    # NOT respawn=True: launch respawned it AFTER Ctrl-C while slower nodes
    # were still stopping, again and again, so the launch never exited (Isaac,
    # 2026-10-31) -- in the rover container, every `docker stop` would end in
    # SIGKILL. If it dies, release from a shell: ros2 topic pub --once /e_stop
    # std_msgs/msg/Bool '{data: false}'.
    e_stop = Node(
        package='lunabot_bringup',
        executable='operator_e_stop.py',
        name='operator_e_stop',
        output='log',
    )

    return LaunchDescription(ARGUMENTS + [bridge, camera, e_stop])
