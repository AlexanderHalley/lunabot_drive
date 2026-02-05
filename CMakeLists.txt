cmake_minimum_required(VERSION 3.8)
project(lunabot_drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)

add_executable(minimal_drive src/minimal_drive.cpp)
ament_target_dependencies(minimal_drive rclcpp sensor_msgs)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node rclcpp geometry_msgs sensor_msgs nav_msgs tf2_ros tf2_geometry_msgs)
target_link_libraries(drive_node sparkcan::sparkcan)

install(TARGETS
  minimal_drive
  drive_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs description DESTINATION share/${PROJECT_NAME})
install(PROGRAMS nodes/bandwidth_monitor.py DESTINATION lib/${PROJECT_NAME})

ament_package()
