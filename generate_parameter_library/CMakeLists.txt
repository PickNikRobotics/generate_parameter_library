cmake_minimum_required(VERSION 3.22)
project(generate_parameter_library NONE)

# find dependencies
find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "generate_parameter_library-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
