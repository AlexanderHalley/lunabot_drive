cmake_minimum_required(VERSION 3.8)
project(lunabot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # Pure YAML: the footprint against the chassis in properties.xacro, the
  # velocity limits against diff_drive_controller's, the observation ranges
  # against the camera's depth clip, and the stamped-command chain against
  # twist_mux. Every one of those was a comment saying "keep this in sync".
  #
  # Needs no ROS and no Nav2, so CI runs it standalone as well.
  ament_add_pytest_test(test_nav2_params test/test_nav2_params.py)

  # Constructs the Nav2 launch description without running it. The check that
  # earns its keep is that no Nav2 node publishes to /cmd_vel, which belongs
  # to twist_mux -- which is exactly what nav2_bringup's own launch file does.
  ament_add_pytest_test(test_navigation_launch test/test_navigation_launch.py)
endif()

ament_package()
