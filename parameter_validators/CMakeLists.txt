cmake_minimum_required(VERSION 3.22)
project(parameter_validators)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)

add_library(parameter_validators INTERFACE)
target_include_directories(parameter_validators INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(parameter_validators INTERFACE cxx_std_17)
target_link_libraries(parameter_validators
  INTERFACE
    fmt::fmt
    rclcpp::rclcpp
)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS parameter_validators
  EXPORT export_parameter_validators
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_parameter_validators)
ament_export_dependencies(fmt rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linters
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
