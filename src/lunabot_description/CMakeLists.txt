cmake_minimum_required(VERSION 3.8)
project(lunabot_description)

find_package(ament_cmake REQUIRED)

# Description is data-only: install the xacro/urdf, meshes, launch and rviz.
install(DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
