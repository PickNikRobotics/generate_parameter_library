cmake_minimum_required(VERSION 3.20)
project(generate_parameter_library_py)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

ament_get_python_install_dir(_PY_INSTALL_DIR)

# Install the python package directory
install(
  DIRECTORY generate_parameter_library_py/
  DESTINATION "${_PY_INSTALL_DIR}/generate_parameter_library_py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "test" EXCLUDE
)

# Install console entrypoints as wrapper executables
install(PROGRAMS
  scripts/generate_parameter_library_cpp
  scripts/generate_parameter_library_python
  scripts/generate_parameter_library_markdown
  DESTINATION lib/${PROJECT_NAME}
)

# Standard ROS package installs
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install test YAMLs
install(
  DIRECTORY generate_parameter_library_py/test/
  DESTINATION share/${PROJECT_NAME}/test
)

find_package(hydra_cpack REQUIRED)

ament_package()

hydra_cpack_ros(VERSION ${HYDRA_CPACK_VERSION})
