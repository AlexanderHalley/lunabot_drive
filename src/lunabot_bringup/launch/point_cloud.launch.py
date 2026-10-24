# Copyright 2027 Lunabot
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""/oak_d/points from the depth image: decimated, clipped. Robot AND sim.

One definition, so the costmaps and the boulder detector see the same cloud
in Isaac as on the robot. camera.launch.py includes it on the Pi; under
hw:=sim, robot.launch.py does, and Isaac publishes no cloud of its own.

rtabmap_util's point_cloud_xyz, not depth_image_proc: Jazzy's
point_cloud_xyz_node (5.0) has no `decimation` or `approximate_sync`
parameter and ignores both silently -- the robot was asking for a 4x
decimated cloud and getting all 256k points. Isaac's own cloud was the same
256k points at every rendered frame: 3 MB, ~50 Hz, best effort. C++
subscribers then lost clouds whole, and the boulder detector published
nothing for minutes at a time.

Accepts 16UC1 millimetres (the OAK-D) and 32FC1 metres (Isaac) alike.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use /clock.'),
    DeclareLaunchArgument(
        'decimation',
        default_value='4',
        description='Keep every Nth pixel in each axis. 4 means 16x fewer points.',
    ),
    DeclareLaunchArgument(
        'max_range',
        default_value='2.0',
        description='Discard points beyond this range, metres. Matches the driver clip.',
    ),
    DeclareLaunchArgument('depth_topic', default_value='/oak_d/stereo/image_raw'),
    DeclareLaunchArgument('camera_info_topic', default_value='/oak_d/stereo/camera_info'),
    DeclareLaunchArgument('cloud_topic', default_value='/oak_d/points'),
]


def generate_launch_description():
    cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='point_cloud_xyz',
        parameters=[
            {
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
                'decimation': ParameterValue(LaunchConfiguration('decimation'), value_type=int),
                'max_depth': ParameterValue(LaunchConfiguration('max_range'), value_type=float),
                'min_depth': 0.0,
                'voxel_size': 0.0,
                # The depth image and its camera_info do not always carry
                # identical stamps off the device. Exact sync drops everything.
                'approx_sync': True,
            }
        ],
        remappings=[
            ('depth/image', LaunchConfiguration('depth_topic')),
            ('depth/camera_info', LaunchConfiguration('camera_info_topic')),
            ('cloud', LaunchConfiguration('cloud_topic')),
        ],
        output='log',
    )
    return LaunchDescription(ARGUMENTS + [cloud])
