# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""rtabmap backend: RGB-D SLAM on CPU.

The default, and the one that actually runs today. Works on a Pi, works on a
laptop, works against a recorded bag, needs no GPU.

Owns `map -> odom`. Owns `odom -> base_link` only when publish_odom_tf:=true,
which happens with odom_source:=visual.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# rgbd_odometry as an EKF input: never publishes TF (the EKF owns
# odom -> base_link), and names its frame so it cannot be mistaken for odom.
VO_FOR_EKF_PARAMETERS = {'publish_tf': False, 'odom_frame_id': 'odom_visual'}

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock.'),
    DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Localise against the existing database instead of mapping.',
    ),
    DeclareLaunchArgument(
        'publish_odom_tf',
        default_value='false',
        description='Run rgbd_odometry and let it publish odom -> base_link.',
    ),
    DeclareLaunchArgument(
        'visual_odom_topic',
        default_value='',
        description=(
            'If set (and publish_odom_tf is false), also run rgbd_odometry WITHOUT '
            'TF, publishing here, for the EKF to fuse slip-free velocity.'
        ),
    ),
    DeclareLaunchArgument(
        'rgb_topic', default_value='/oak_d/rgb/image_rect', description='Rectified RGB.'
    ),
    DeclareLaunchArgument(
        'depth_topic',
        default_value='/oak_d/stereo/image_raw',
        description='Depth, registered to the RGB frame.',
    ),
    DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/oak_d/rgb/camera_info',
        description='Intrinsics for rgb_topic.',
    ),
    DeclareLaunchArgument(
        'imu_topic',
        default_value='/oak_d/imu/data',
        description='IMU, for the gravity constraint.',
    ),
    DeclareLaunchArgument('odom_topic', default_value='/odom', description='Wheel odometry.'),
    DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lunabot_slam'), 'config', 'rtabmap.yaml']
        ),
        description='rtabmap parameter file.',
    ),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config = LaunchConfiguration('config')

    # rtabmap's own topic names on the left, ours on the right. This mapping
    # is the entire interface between the contract and the backend.
    remappings = [
        ('rgb/image', LaunchConfiguration('rgb_topic')),
        ('depth/image', LaunchConfiguration('depth_topic')),
        ('rgb/camera_info', LaunchConfiguration('camera_info_topic')),
        ('imu', LaunchConfiguration('imu_topic')),
        ('odom', LaunchConfiguration('odom_topic')),
    ]

    common = {
        'parameters': [config, {'use_sim_time': use_sim_time}],
        'remappings': remappings,
        'output': 'screen',
    }

    # Mapping. --delete_db_on_start is what makes each run start from a clean
    # map; without it rtabmap silently resumes from its database and
    # yesterday's session merges into today's.
    mapping = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        condition=UnlessCondition(LaunchConfiguration('localization')),
        arguments=['--delete_db_on_start'],
        **common,
    )

    # Localisation against an existing database. Same node, memory frozen.
    localization = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        condition=IfCondition(LaunchConfiguration('localization')),
        parameters=[
            config,
            {
                'use_sim_time': use_sim_time,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
            },
        ],
        remappings=remappings,
        output='screen',
    )

    # Visual odometry, only when it is the odom source. With wheel odometry
    # this node must NOT run: it would publish odom -> base_link alongside
    # diff_drive_controller, and two publishers on one transform gives a tree
    # that looks fine in view_frames and behaves nondeterministically.
    visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        condition=IfCondition(LaunchConfiguration('publish_odom_tf')),
        parameters=[
            config,
            {'use_sim_time': use_sim_time, 'publish_tf': True, 'odom_frame_id': 'odom'},
        ],
        remappings=remappings,
        output='screen',
    )

    # Visual odometry as an EKF INPUT (odom_source:=ekf): velocity from the
    # camera, which does not slip. Wheels do -- in sand, by tens of percent
    # at cusps and stalls -- and the EKF's position drifted by a metre over
    # three arena goals on wheel speed alone. publish_tf is forced off and the
    # frame renamed: the EKF owns odom -> base_link, and this must never be
    # mistaken for it.
    fusion_remappings = [
        (old, LaunchConfiguration('visual_odom_topic') if old == 'odom' else new)
        for old, new in remappings
    ]
    visual_odometry_for_ekf = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('visual_odom_topic'),
                    "' != '' and '",
                    LaunchConfiguration('publish_odom_tf'),
                    "' != 'true'",
                ]
            )
        ),
        parameters=[
            config,
            {'use_sim_time': use_sim_time, **VO_FOR_EKF_PARAMETERS},
        ],
        remappings=fusion_remappings,
        output='screen',
    )

    return LaunchDescription(
        ARGUMENTS + [mapping, localization, visual_odometry, visual_odometry_for_ekf]
    )
