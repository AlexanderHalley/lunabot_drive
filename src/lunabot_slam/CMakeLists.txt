cmake_minimum_required(VERSION 3.8)
project(lunabot_slam)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
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
  # Constructs every backend's launch description without executing it. This
  # is the only automated coverage the cuVSLAM path can have -- the node may
  # not even be installable -- and it still catches typos and missing
  # substitutions, which is most of what breaks in launch files.
  ament_add_pytest_test(test_slam_launch test/test_slam_launch.py)
endif()

ament_package()
