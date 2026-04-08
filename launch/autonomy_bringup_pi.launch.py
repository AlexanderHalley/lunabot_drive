#!/usr/bin/env python3
"""
Autonomy Bringup — Pi side (run on lunapi).

Everything that needs direct hardware access or low-latency control loops
lives here.  The PC side (autonomy_bringup_pc.launch.py) handles only
joystick teleop, whose commands arrive over the LAN into cmd_vel_mux.

Stack launched here
-------------------
  1.  drive_node              — SparkFlex CAN motors, /odom, /joint_states
  2.  robot_state_publisher   — URDF → TF (base_link, wheels, camera)
  3.  ekf_filter_node         — fuses /odom + IMU → /odometry/filtered, odom->base_link TF
  4.  OAK-D S2 camera         — RGB (AprilTag input) + pointcloud (obstacle costmap)
  5.  apriltag_node           — apriltag_ros: detects tags, publishes camera→tag TF
  6.  apriltag_localizer_node — our bridge: derives and broadcasts map→odom TF
  7.  Nav2 full stack         — controller_server, planner_server, behavior_server,
                                bt_navigator, lifecycle_manager
  8.  cmd_vel_mux             — Nav2 (priority 1) vs teleop joystick (priority 10)
                                output → /cmd_vel → drive_node
  9.  bucket actuators        — lift + tilt ActuatorDriverNode instances
  10. mission_state_node      — autonomy state machine + excavation/deposition sequences
  11. bandwidth/health/wifi monitors
  12. rosbridge_websocket     — dashboard.html on dreamfyre connects here (ws://lunapi:9090)

Pre-requisites
--------------
  - CAN bus initialised:          sudo ./scripts/initialise_can
  - apriltag_ros installed:       sudo apt install ros-jazzy-apriltag-ros
  - Nav2 installed:               sudo apt install ros-jazzy-navigation2
  - rosbridge installed:          sudo apt install ros-jazzy-rosbridge-suite

On dreamfyre, also launch:
  ros2 launch lunabot_drive autonomy_bringup_pc.launch.py

Usage
-----
  ros2 launch lunabot_drive autonomy_bringup_pi.launch.py
  ros2 launch lunabot_drive autonomy_bringup_pi.launch.py dig_x:=3.5 dig_y:=2.0
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('lunabot_drive')

    ekf_params      = os.path.join(pkg, 'config', 'params', 'ekf_params.yaml')
    nav2_params     = os.path.join(pkg, 'config', 'params', 'nav2_params.yaml')
    apriltag_params = os.path.join(pkg, 'config', 'params', 'apriltag_localizer_params.yaml')
    mux_params      = os.path.join(pkg, 'config', 'cmd_vel_mux_lunabot.yaml')
    pc_config       = os.path.join(pkg, 'config', 'oak_d_pointcloud_only.yaml')
    urdf_file       = os.path.join(pkg, 'description', 'robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str
    )

    wheel_radius = 0.1778   # 7 in radius (14 in diameter)
    wheel_base   = 0.762    # distance between left and right wheels (2 * wheel_offset_y)

    # Waypoint arguments — adjust at the field after surveying the arena
    dig_x    = LaunchConfiguration('dig_x')
    dig_y    = LaunchConfiguration('dig_y')
    hopper_x = LaunchConfiguration('hopper_x')
    hopper_y = LaunchConfiguration('hopper_y')
    home_x   = LaunchConfiguration('home_x')
    home_y   = LaunchConfiguration('home_y')

    return LaunchDescription([

        DeclareLaunchArgument('dig_x',    default_value='4.0',
                              description='Dig zone X in map frame (m)'),
        DeclareLaunchArgument('dig_y',    default_value='2.5',
                              description='Dig zone Y in map frame (m)'),
        DeclareLaunchArgument('hopper_x', default_value='0.5',
                              description='ISRU hopper X in map frame (m)'),
        DeclareLaunchArgument('hopper_y', default_value='2.5',
                              description='ISRU hopper Y in map frame (m)'),
        DeclareLaunchArgument('home_x',   default_value='0.3',
                              description='Home pose X in map frame (m)'),
        DeclareLaunchArgument('home_y',   default_value='2.5',
                              description='Home pose Y in map frame (m)'),

        # ── 1. Drive node (EKF active → drive_node must NOT publish odom TF) ──
        Node(
            package='lunabot_drive',
            executable='drive_node',
            name='drive_node',
            parameters=[{
                'can_interface':    'can0',
                'left_front_id':    2,
                'right_front_id':   1,
                'left_rear_id':     3,
                'right_rear_id':    4,
                'wheel_base':       wheel_base,
                'wheel_radius':     wheel_radius,
                'gear_ratio':       100.0,
                'max_duty_cycle':   0.8,
                'joint_state_rate': 50.0,
                'publish_odom_tf':  False,   # EKF publishes odom→base_link
            }],
            output='screen',
        ),

        # ── 2. Robot state publisher ──────────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # ── 3. EKF — wheel odom + camera IMU → /odometry/filtered ────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
        ),

        # ── 4. OAK-D S2 camera ───────────────────────────────────────────────
        #    oak_d_pointcloud_only.yaml uses an RGBD pipeline: stereo depth
        #    (for the Nav2 obstacle costmap) plus 5 fps / 720p RGB (for
        #    apriltag_ros). Total bandwidth stays well under 4 Mbps.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'oak_d_camera.launch.py'])
            ),
            launch_arguments={'config': pc_config}.items(),
        ),

        # ── 5. apriltag_ros — detects tags and publishes camera→tag TF ───────
        #
        # IMPORTANT — pre-competition calibration checklist:
        #   1. `size` must match the ACTUAL BLACK BORDER of your printed tag in
        #      metres. Mis-measured size causes a pure translation scale error
        #      in the derived pose. Use a caliper or mm ruler on the final print.
        #   2. `image_rect` / `camera_info` topics must exist. With our
        #      oak_d_pointcloud_only.yaml config they are published at 5 Hz /
        #      720p — confirm with `ros2 topic hz /oak/rgb/image_raw` before
        #      every field test.
        #   3. The tag detection TF child frames are `tag36h11:<id>`, parented
        #      to whichever optical frame depthai publishes (by default
        #      `oak_rgb_camera_optical_frame` when the depthai node is named
        #      `oak`). If you rename the camera node, update
        #      apriltag_localizer_params.yaml `camera_frame` to match.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[{
                'family':                     'tag36h11',
                'size':                        0.16,    # metres — MEASURE YOUR ACTUAL TAGS
                'max_hamming':                 0,
                'detector.threads':            2,       # Pi 5 has 4 cores
                'detector.quad_decimate':      2.0,     # trade accuracy for speed
                'detector.quad_sigma':         0.0,
                'detector.refine_edges':       True,
                'detector.decode_sharpening':  0.25,
            }],
            remappings=[
                ('image_rect',  '/oak/rgb/image_raw'),
                ('camera_info', '/oak/rgb/camera_info'),
            ],
            output='screen',
        ),

        # ── 6. AprilTag localizer — map→odom TF from tag detections ──────────
        Node(
            package='lunabot_drive',
            executable='apriltag_localizer_node',
            name='apriltag_localizer',
            parameters=[apriltag_params],
            output='screen',
        ),

        # ── 7. Nav2 stack ─────────────────────────────────────────────────────
        #    controller_server cmd_vel is remapped into cmd_vel_mux so that
        #    joystick teleop from the PC can pre-empt Nav2 at any time.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[nav2_params],
            remappings=[('cmd_vel', '/cmd_vel_mux/input/navigation')],
            output='screen',
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            parameters=[nav2_params],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                ],
            }],
            output='screen',
        ),

        # ── 8. cmd_vel mux ────────────────────────────────────────────────────
        #    Joystick (priority 10) beats Nav2 (priority 1).
        #    Joystick topics arrive from dreamfyre over the LAN via ROS_DOMAIN_ID=42.
        Node(
            package='cmd_vel_mux',
            executable='cmd_vel_mux_node',
            name='cmd_vel_mux',
            parameters=[mux_params],
            output='screen',
        ),

        # ── 9. Bucket actuators ───────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg, 'launch', 'bucket_bringup.launch.py'])
            ),
        ),

        # ── 10. Mission state node ────────────────────────────────────────────
        Node(
            package='lunabot_drive',
            executable='mission_state_node',
            name='mission_state_node',
            parameters=[{
                'dig_x':              dig_x,
                'dig_y':              dig_y,
                'hopper_x':           hopper_x,
                'hopper_y':           hopper_y,
                'home_x':             home_x,
                'home_y':             home_y,
                'nav_timeout_s':      120.0,
                'service_timeout_s':   35.0,
                'dump_wait_s':           3.0,
                'lift_actuator_ns':  '/bucket/lift/lift_driver',
                'tilt_actuator_ns':  '/bucket/tilt/tilt_driver',
            }],
            output='screen',
        ),

        # ── 11. Dashboard support nodes ───────────────────────────────────────
        Node(
            package='lunabot_drive',
            executable='bandwidth_monitor_node',
            name='bandwidth_monitor_node',
            output='screen',
        ),
        Node(
            package='lunabot_drive',
            executable='health_monitor_node',
            name='health_monitor_node',
            output='screen',
        ),
        Node(
            package='lunabot_drive',
            executable='wifi_monitor_node',
            name='wifi_monitor_node',
            output='screen',
        ),

        # ── 12. rosbridge — dashboard.html on dreamfyre connects here ─────────
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            parameters=[{'port': 9090}],
            output='screen',
        ),

        # ── 13. web_video_server — MJPEG stream for dashboard camera feed ────
        #    Streams /oak/rgb/image_raw as MJPEG over HTTP on port 8080.
        #    The Pi is the subscriber (no LAN overhead until a browser requests
        #    the stream); the dashboard toggles it via an <img src> swap.
        #    Install: sudo apt install ros-jazzy-web-video-server
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{
                'port': 8080,
                'default_stream_type': 'mjpeg',
            }],
            output='screen',
        ),
    ])
