cmake_minimum_required(VERSION 3.8)
project(lunabot_simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config isaac
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

ament_package()
