cmake_minimum_required(VERSION 3.8)
project(lunabot_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
