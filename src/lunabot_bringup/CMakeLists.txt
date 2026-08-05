cmake_minimum_required(VERSION 3.8)
project(lunabot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Constructs every launch description without executing anything. Catches
  # typos, missing substitutions and bad argument names in about a second --
  # which is most of what actually breaks in launch files.
  ament_add_pytest_test(test_launch_descriptions test/test_launch_descriptions.py)

  # Structural checks on the YAML: covariance matrix shapes, the wheel-name
  # contract with the URDF, and the YAML 1.1 exponent trap that makes `1.0e6`
  # parse as a string. Both ekf.yaml and controllers.yaml shipped with that
  # bug before this test existed.
  ament_add_pytest_test(test_config_files test/test_config_files.py)

  # Brings the whole stack up on mock hardware and drives it. No hardware, no
  # GPU, no network. This is the highest-value test in the workspace: if
  # /odom moves, the URDF parsed, the plugin loaded, the interfaces were
  # claimed, the kinematics ran and TF is connected.
  add_launch_test(test/test_mock_bringup.py TIMEOUT 120)
endif()

ament_package()
