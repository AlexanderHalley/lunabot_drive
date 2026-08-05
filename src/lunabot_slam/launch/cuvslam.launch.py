"""cuVSLAM backend: GPU stereo visual-inertial SLAM.

SCAFFOLDING. This is not known to run -- see docs/SLAM.md and the header of
config/cuvslam.yaml. It exists so the launch plumbing, topic contract and
frame ownership are already correct when the 2027 compute decision is made,
and so CI can prove the launch file at least constructs.

Consumes a RECTIFIED STEREO PAIR, not RGB + depth. Start the camera with
profile:=stereo_rect.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock.'),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localise against a saved map instead of mapping.',
    ),
    DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Publish odom -> base_link. True only with odom_source:=visual.',
    ),
    DeclareLaunchArgument(
        'left_rect_topic',
        default_value='/oak_d/left/image_rect',
        description='Rectified left mono image.',
    ),
    DeclareLaunchArgument(
        'right_rect_topic',
        default_value='/oak_d/right/image_rect',
        description='Rectified right mono image.',
    ),
    DeclareLaunchArgument(
        'left_info_topic',
        default_value='/oak_d/left/camera_info',
        description='Intrinsics for left_rect_topic.',
    ),
    DeclareLaunchArgument(
        'right_info_topic',
        default_value='/oak_d/right/camera_info',
        description='Intrinsics for right_rect_topic.',
    ),
    DeclareLaunchArgument('imu_topic', default_value='/oak_d/imu/data', description='IMU.'),
    DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Wheel odometry. Not consumed by cuVSLAM; declared for interface parity.',
    ),
    DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_slam'), 'config', 'cuvslam.yaml']
        ),
        description='cuVSLAM parameter file.',
    ),
]


def generate_launch_description():
    # ==================== VERIFY THESE ====================
    # Isaac ROS changed its visual SLAM topic naming around 3.x, from
    # stereo_camera/left/image to a multi-camera image_0 / camera_info_0
    # scheme. Both are plausible depending on the release you install.
    #
    # Check before assuming the node is broken:
    #   ros2 node info /visual_slam_node
    #
    # The multi-camera scheme is used here because it is the newer one; if
    # your release wants the older names, change them here and only here.
    # ======================================================
    remappings = [
        ('visual_slam/image_0', LaunchConfiguration('left_rect_topic')),
        ('visual_slam/camera_info_0', LaunchConfiguration('left_info_topic')),
        ('visual_slam/image_1', LaunchConfiguration('right_rect_topic')),
        ('visual_slam/camera_info_1', LaunchConfiguration('right_info_topic')),
        ('visual_slam/imu', LaunchConfiguration('imu_topic')),
    ]

    visual_slam = Node(
        package='isaac_ros_visual_slam',
        executable='isaac_ros_visual_slam',
        name='visual_slam_node',
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Launch argument wins over the YAML, so odom TF ownership is
                # decided in one place for every backend.
                'publish_odom_to_base_tf': LaunchConfiguration('publish_odom_tf'),
            },
        ],
        remappings=remappings,
        output='screen',
    )

    return LaunchDescription(
        ARGUMENTS
        + [
            LogInfo(
                msg=(
                    'cuVSLAM backend selected. This path is scaffolding: it needs an '
                    'NVIDIA GPU and an Isaac ROS release for this distro. If the node '
                    'fails to start, that is expected -- see docs/SLAM.md.'
                )
            ),
            visual_slam,
        ]
    )
