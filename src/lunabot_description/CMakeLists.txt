cmake_minimum_required(VERSION 3.8)
project(lunabot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz
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
  # Expands the xacro for every `hardware` value and checks the resulting
  # tree against docs/TOPIC_FRAME_CONTRACT.md. Catches the failure mode
  # where the model is valid XML, valid URDF, and wired to the wrong
  # plugin or missing a frame something downstream needs.
  ament_add_pytest_test(test_urdf_parses test/test_urdf_parses.py)
endif()

ament_package()
