cmake_minimum_required(VERSION 3.16)
project(lunabot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  # /drive/status: the message, and the publisher that can be called from
  # write() without allocating or blocking the control loop.
  lunabot_msgs
  realtime_tools
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# Built from source via lunabot.repos -- not in the rosdep index.
find_package(sparkcan REQUIRED)

# A SHARED library, not an executable. pluginlib dlopens this at runtime when
# controller_manager encounters the plugin name in the URDF; there is no main().
add_library(lunabot_hardware SHARED
  src/spark_flex_system.cpp
  src/spark_flex_motor.cpp
)

target_compile_features(lunabot_hardware PUBLIC cxx_std_17)
target_include_directories(lunabot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lunabot_hardware>
)
ament_target_dependencies(lunabot_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(lunabot_hardware PUBLIC sparkcan::sparkcan)

target_compile_definitions(lunabot_hardware PRIVATE "LUNABOT_HARDWARE_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface lunabot_hardware_plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/lunabot_hardware
)
install(
  TARGETS lunabot_hardware
  EXPORT export_lunabot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

  find_package(ament_cmake_gmock REQUIRED)

  # pluginlib-loads the plugin against a synthetic URDF and exercises
  # on_init. No SocketCAN, so this runs in CI on a machine with no CAN stack
  # and no motors -- which is the only automated coverage the real hardware
  # path can have.
  ament_add_gmock(test_load_spark_flex_system test/test_load_spark_flex_system.cpp)
  target_include_directories(test_load_spark_flex_system PRIVATE include)
  target_link_libraries(test_load_spark_flex_system lunabot_hardware)
  ament_target_dependencies(test_load_spark_flex_system
    hardware_interface
    pluginlib
    # The fixture builds a node to hand ResourceManager a clock and a logger.
    rclcpp
  )
endif()

ament_export_targets(export_lunabot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
