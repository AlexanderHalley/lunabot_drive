cmake_minimum_required(VERSION 3.8)
project(lunabot_excavation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/excavator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ruff owns Python, via pre-commit; see CONTRIBUTING.md, "One linter per language".
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  # The dig and dump sequences, cancel / e-stop / fault, on the model
  # backend. Pure Python: the node imports rclpy only in main().
  ament_add_pytest_test(test_excavator test/test_excavator.py)
  # The server against the real interfaces: goals, cancel, e-stop, stow.
  ament_add_pytest_test(test_excavator_node test/test_excavator_node.py TIMEOUT 120)
  # Its launch arguments clash with no other launch file (they are shared down includes).
  ament_add_pytest_test(test_excavation_launch test/test_excavation_launch.py)
endif()

ament_package()
