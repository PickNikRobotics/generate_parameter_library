cmake_minimum_required(VERSION 3.20)
project(generate_parameter_library NONE)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Export CMake helper modules for downstream packages.
install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(
  CONFIG_EXTRAS "generate_parameter_library-extras.cmake"
)

# Add hydra_cpack for deb packaging
find_package(hydra_cpack REQUIRED)

if(NOT DEFINED HYDRA_CPACK_VERSION)
  message(FATAL_ERROR "HYDRA_CPACK_VERSION must be set for deb packaging.")
endif()

hydra_cpack_ros(VERSION ${HYDRA_CPACK_VERSION})
