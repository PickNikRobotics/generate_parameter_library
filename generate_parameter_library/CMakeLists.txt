cmake_minimum_required(VERSION 3.20)
project(generate_parameter_library NONE)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cmake
  DESTINATION share/generate_parameter_library
)

ament_package(
  CONFIG_EXTRAS "generate_parameter_library-extras.cmake"
)
