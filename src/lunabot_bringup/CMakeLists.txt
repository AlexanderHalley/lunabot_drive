cmake_minimum_required(VERSION 3.8)
project(lunabot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# lib/, not share/: this is where `ros2 run lunabot_bringup check_stack.py`
# looks. PROGRAMS rather than FILES so it stays executable.
install(
  PROGRAMS scripts/check_stack.py
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
  #
  # ENV ROS_DOMAIN_ID: the two launch tests get a domain each, and neither is
  # the default. ctest runs them back to back, and /robot_description is
  # TRANSIENT_LOCAL -- so the previous stack's robot_state_publisher, still
  # winding down, served its latched description to the next test's
  # subscriber. The sim test read `mock_components/GenericSystem` off a
  # correctly-configured sim stack and failed. Anything latched can do this;
  # separate domains end the whole class rather than that one symptom.
  add_launch_test(test/test_mock_bringup.py TIMEOUT 120 ENV ROS_DOMAIN_ID=71)

  # The same thing on hw:=sim, against test/isaac_double.py standing in for
  # Isaac's bridge graphs. Still no GPU and no Isaac install: what it covers
  # is the plumbing that differs between mock and sim -- the TopicBasedSystem
  # plugin, the two /isaac topics, use_sim_time, and wrapped joint positions.
  #
  # Longer timeout than the mock test: it waits for /clock before starting
  # anything, then drives for five seconds to cross the joint wrap several
  # times.
  add_launch_test(test/test_sim_bringup.py TIMEOUT 300 ENV ROS_DOMAIN_ID=72)
endif()

ament_package()
