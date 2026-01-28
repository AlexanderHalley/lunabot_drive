cmake_minimum_required(VERSION 3.8)
project(lunabot_drive)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sparkcan REQUIRED)

add_executable(drive_node src/drive_node.cpp)
ament_target_dependencies(drive_node rclcpp geometry_msgs nav_msgs tf2_geometry_msgs)
target_link_libraries(drive_node sparkcan::sparkcan)

install(TARGETS drive_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs urdf DESTINATION share/${PROJECT_NAME})

ament_package()
