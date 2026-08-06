# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""OAK-D S2 driver, plus host-side point cloud generation.

Runs on the robot. In simulation this file is not used at all -- Isaac's
ROS2CameraHelper graphs publish the same topic names, which is the point of
the contract.

The `profile` argument selects a camera configuration, not just a set of
remappings. The SLAM backends genuinely need different camera setups: rtabmap
wants RGB plus depth registered into the RGB frame, cuVSLAM wants a rectified
stereo pair and no depth at all.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from Isaac Sim instead of the wall clock.',
    ),
    DeclareLaunchArgument(
        'profile',
        default_value='default',
        choices=['default', 'rgb_only', 'pointcloud', 'stereo_rect'],
        description=(
            'Camera configuration. default = RGB + aligned depth + IMU (rtabmap). '
            'stereo_rect = rectified stereo pair, no depth (cuVSLAM). '
            'pointcloud = depth only, no RGB. rgb_only = calibration and triage.'
        ),
    ),
    DeclareLaunchArgument(
        'point_cloud',
        default_value='true',
        description=(
            'Generate /oak_d/points on the host from the depth image. '
            'Ignored for profiles that publish no depth (rgb_only, stereo_rect).'
        ),
    ),
    DeclareLaunchArgument(
        'point_cloud_decimation',
        default_value='4',
        description='Keep every Nth pixel in each axis. 4 means 16x fewer points.',
    ),
    DeclareLaunchArgument(
        'point_cloud_max_range',
        default_value='2.0',
        description='Discard points beyond this range, metres. Matches the driver clip.',
    ),
]

# Profile -> config file. Indirection so the file layout can change without
# breaking every command anyone has written down.
# ==================== THIS NAME IS THE FRAME PREFIX ====================
# Not a parameter. depthai_ros_driver derives every published frame_id from
# the node's name, in sensor_helpers.cpp::tfPrefix():
#
#     if (camera.i_publish_tf_from_calibration)  return camera.i_tf_base_frame;
#     return node->get_name();
#
# We run with i_publish_tf_from_calibration false, because
# robot_state_publisher owns the TF tree -- so this string is the only reason
# the driver's frames are oak_d_* and match the URDF. Rename it and every
# camera topic quietly carries frame_ids nothing in the tree has heard of:
# no error, and rtabmap simply never receives a usable transform.
#
# Verified against depthai-ros 2.12.2, the version Jazzy ships. A module
# constant rather than a literal so a test can pin it without reaching into
# launch_ros internals -- Node.node_name is unreadable until the action runs.
# ======================================================================
DRIVER_NODE_NAME = 'oak_d'

CONFIG_FOR_PROFILE = {
    'default': 'oak_d_s2.yaml',
    'rgb_only': 'oak_d_s2_rgb_only.yaml',
    'pointcloud': 'oak_d_s2_pointcloud.yaml',
    'stereo_rect': 'oak_d_s2_stereo_rect.yaml',
}

# Only these profiles publish a depth image for depth_image_proc to consume.
PROFILES_WITH_DEPTH = {'default', 'pointcloud'}


def _nodes(context, *args, **kwargs):
    """Build the node list, at launch time, because the profile decides which nodes exist.

    An OpaqueFunction rather than IfCondition gymnastics: choosing a filename
    from a dict and deciding whether a node runs are both trivial in Python
    and genuinely awkward as nested substitutions.
    """
    profile = LaunchConfiguration('profile').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    config = str(
        Path(get_package_share_directory('lunabot_bringup'))
        / 'config'
        / CONFIG_FOR_PROFILE[profile]
    )

    nodes = [
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name=DRIVER_NODE_NAME,
            parameters=[config, {'use_sim_time': use_sim_time}],
            output='screen',
        )
    ]

    want_cloud = LaunchConfiguration('point_cloud').perform(context) == 'true'
    if want_cloud and profile in PROFILES_WITH_DEPTH:
        # Built on the robot rather than shipped as a depth image and
        # converted offboard. Counter-intuitive but correct: decimated points
        # clipped at 2 m are a fraction of the raw depth image's bytes, so
        # this trades Pi CPU for network bandwidth, which is scarcer.
        nodes.append(
            Node(
                package='depth_image_proc',
                executable='point_cloud_xyz_node',
                name='point_cloud_xyz',
                remappings=[
                    ('image_rect', '/oak_d/stereo/image_raw'),
                    ('camera_info', '/oak_d/stereo/camera_info'),
                    ('points', '/oak_d/points'),
                ],
                parameters=[
                    {
                        'queue_size': 10,
                        # The depth image and its camera_info do not always
                        # carry identical stamps off the device. Exact sync
                        # drops everything; approximate sync works.
                        'approximate_sync': True,
                        'decimation': int(
                            LaunchConfiguration('point_cloud_decimation').perform(context)
                        ),
                        'min_range': 0.0,
                        'max_range': float(
                            LaunchConfiguration('point_cloud_max_range').perform(context)
                        ),
                        'use_sim_time': use_sim_time,
                    }
                ],
                output='log',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(ARGUMENTS + [OpaqueFunction(function=_nodes)])
