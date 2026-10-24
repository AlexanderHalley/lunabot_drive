cmake_minimum_required(VERSION 3.8)
project(lunabot_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Costmap layers, loaded by nav2_costmap_2d through pluginlib
# (costmap_plugins.xml). UnseenShadowLayer: the unseen space behind a seen
# obstacle surface is obstacle -- docs/NAVIGATION.md.
add_library(lunabot_costmap_layers SHARED src/unseen_shadow_layer.cpp)
target_compile_features(lunabot_costmap_layers PUBLIC cxx_std_17)
target_include_directories(lunabot_costmap_layers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lunabot_navigation>
)
ament_target_dependencies(lunabot_costmap_layers PUBLIC nav2_costmap_2d pluginlib rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(
  TARGETS lunabot_costmap_layers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/lunabot_navigation)

install(
  DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/curvature_limiter.py scripts/initial_survey.py scripts/mission.py
    scripts/fiducial_localizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ruff owns Python, clang-format owns C++, both via pre-commit. Skip the
  # three ament linters that cover the same files with different opinions --
  # running both undoes each other. Everything else in ament_lint_common still
  # runs: copyright, cpplint, cppcheck, lint_cmake, xmllint.
  # See CONTRIBUTING.md, "One linter per language".
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # The shadow rule on small hand-drawn grids. No ROS, no costmap.
  ament_add_gtest(test_unseen_shadow test/test_unseen_shadow.cpp)
  target_include_directories(test_unseen_shadow PRIVATE include)

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

  # The curvature limiter's clamp, and that it never commands tighter than the
  # planner plans. Pure Python; the node imports rclpy only in main().
  ament_add_pytest_test(test_curvature_limiter test/test_curvature_limiter.py)

  # The mission's step list, and that its UCF poses sit in the right zones.
  ament_add_pytest_test(test_mission test/test_mission.py)

  # The start fix from the wall AprilTags, on synthetic renders of a spin.
  ament_add_pytest_test(test_fiducial_localizer test/test_fiducial_localizer.py)
endif()

ament_package()
