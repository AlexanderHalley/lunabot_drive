"""
Bucket actuator bringup launch file.

Launches two ActuatorDriverNode instances under distinct names so logs,
lifecycle messages, and the node health monitor can tell them apart:

  /bucket/lift/lift_driver  (lift params)
  /bucket/tilt/tilt_driver  (tilt params)

Dashboard remaps
----------------
The dashboard (dashboard/dashboard.html) subscribes to
  /bucket/lift_position   (std_msgs/Float32)
  /bucket/tilt_position   (std_msgs/Float32)
  /bucket/state           (std_msgs/String)

We remap the lift driver's ~/position to /bucket/lift_position, the tilt
driver's ~/position to /bucket/tilt_position, and the lift driver's
~/state to /bucket/state (the lift is the primary state reported to the
operator — tilt state is visible in the diagnostic topic).

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
        name="lift_driver",
        namespace="bucket/lift",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # Dashboard telemetry topics
            ("~/position", "/bucket/lift_position"),
            ("~/state",    "/bucket/state"),
        ],
        output="screen",
    )

    tilt_node = Node(
        package="lunabot_drive",
        executable="actuator_driver_node",
        name="tilt_driver",
        namespace="bucket/tilt",
        parameters=[
            config_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            # Dashboard telemetry topics (tilt shares the /bucket/state topic
            # with the lift via the DiagnosticStatus; only position is remapped)
            ("~/position", "/bucket/tilt_position"),
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
