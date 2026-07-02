"""Perception bring-up: OAK-D + pointcloud->laserscan + AprilTags.

    ros2 launch lunabot_perception perception.launch.py use_camera:=true

SKELETON. On real hardware this launches the depthai driver (migrate the 2026
oak_d_camera.launch.py here). In Isaac Sim the camera topics come from the bridge,
so launch with use_camera:=false and only run the derived pipelines.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_camera", default_value="true",
                              description="false in sim (camera comes from Isaac bridge)"),
        # TODO: IncludeLaunchDescription depthai_ros_driver (from 2026 oak_d_camera.launch.py)
        #       when use_camera:=true.
        # TODO: pointcloud_to_laserscan node: /camera/depth/points -> /scan.
        # TODO: apriltag_ros node on /camera/image_raw -> /tf (tag frames) for relocalization.
    ])
