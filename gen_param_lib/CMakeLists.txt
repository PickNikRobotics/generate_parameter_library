cmake_minimum_required(VERSION 3.22)
project(gen_param_lib NONE)

# find dependencies
find_package(ament_cmake_core REQUIRED)

ament_package(
  CONFIG_EXTRAS "gen_param_lib-extras.cmake"
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)
