cmake_minimum_required(VERSION 3.20)
project(generate_parameter_library NONE)

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cmake
  DESTINATION share/generate_parameter_library
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_flake8_FOUND TRUE) # Conflicts with black
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "generate_parameter_library-extras.cmake"
)
