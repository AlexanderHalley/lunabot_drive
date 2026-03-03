"""
Bucket actuator bringup launch file.

Launches two ActuatorDriverNode instances:
  /bucket/lift/actuator_driver  (lift params)
  /bucket/tilt/actuator_driver  (tilt params)

Include in hardware_bringup.launch.py:
  from launch.actions import IncludeLaunchDescription
  from launch.launch_description_sources import PythonLaunchDescriptionSource
  from ament_index_python.packages import get_package_share_directory
  import os

  bucket_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('lunabot_drive'),
                       'launch', 'bucket_bringup.launch.py')
      )
  )
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("lunabot_drive")
    config_file = os.path.join(pkg_share, "config", "bucket_actuators.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    lift_node = Node(
        package="lunabot_drive",
        executable="actuator_driver_node",
        name="actuator_driver",
        namespace="bucket/lift",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # node uses ~/command, ~/position, ~/status — namespace resolves these
        ],
        output="screen",
    )

    tilt_node = Node(
        package="lunabot_drive",
        executable="actuator_driver_node",
        name="actuator_driver",
        namespace="bucket/tilt",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            lift_node,
            tilt_node,
        ]
    )
