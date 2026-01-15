cmake_minimum_required(VERSION 3.20)
project(generate_parameter_library NONE)

# find dependencies
find_package(ament_cmake REQUIRED)

# This installs everything under:
#   generate_parameter_library/cmake/ -> <prefix>/share/generate_parameter_library/cmake/
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "generate_parameter_library-extras.cmake"
)
