cmake_minimum_required(VERSION 3.16)
project(lunabot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  visualization_msgs
  vision_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()
find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree)
find_package(Eigen3 REQUIRED)

# The algorithm, with NO ROS dependency. That separation is what lets
# test_cloud_segmentation run in milliseconds with no graph and no bag.
add_library(cloud_segmentation SHARED src/cloud_segmentation.cpp)
target_compile_features(cloud_segmentation PUBLIC cxx_std_17)
target_include_directories(cloud_segmentation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lunabot_perception>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cloud_segmentation PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen)

# The ROS wrapper around it.
add_library(boulder_detector_component SHARED src/boulder_detector_node.cpp)
target_compile_features(boulder_detector_component PUBLIC cxx_std_17)
target_include_directories(boulder_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lunabot_perception>
)
target_link_libraries(boulder_detector_component PUBLIC cloud_segmentation)
ament_target_dependencies(boulder_detector_component PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(boulder_detector_component
  "lunabot_perception::BoulderDetectorNode")

add_executable(boulder_detector src/boulder_detector_main.cpp)
target_link_libraries(boulder_detector PRIVATE boulder_detector_component)
ament_target_dependencies(boulder_detector PUBLIC rclcpp)

add_executable(depth_normalizer src/depth_normalizer_node.cpp)
ament_target_dependencies(depth_normalizer PUBLIC rclcpp sensor_msgs)

install(
  TARGETS cloud_segmentation boulder_detector_component
  EXPORT export_lunabot_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS boulder_detector depth_normalizer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/lunabot_perception)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Python prototyping helpers. Not part of the runtime pipeline -- see the
# scripts' own docstrings.
install(
  PROGRAMS
    scripts/publish_synthetic_cloud.py
    scripts/prototype_cluster.py
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

  find_package(ament_cmake_gtest REQUIRED)
  # Hand-built clouds, no ROS graph, milliseconds. This is where every
  # interesting property of the pipeline is actually checked.
  ament_add_gtest(test_cloud_segmentation test/test_cloud_segmentation.cpp)
  target_link_libraries(test_cloud_segmentation cloud_segmentation)
endif()

ament_export_targets(export_lunabot_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} PCL Eigen3)
ament_package()
