cmake_minimum_required(VERSION 3.22)
project(generate_parameter_library_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(admittance_controller_parameters
  src/parameters.yaml
  include/generate_parameter_library_example/example_validators.hpp
)

add_executable(test_node
  src/main.cpp
)

target_link_libraries(test_node PRIVATE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  admittance_controller_parameters
)

install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_flake8_FOUND TRUE) # Conflicts with black
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_rostest_with_parameters_gtest(test_example_gtest test/example_test_gtest.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/example_params.yaml)
  target_include_directories(test_example_gtest PRIVATE include)
  target_link_libraries(test_example_gtest admittance_controller_parameters)
  ament_target_dependencies(test_example_gtest rclcpp)

  find_package(ament_cmake_gmock REQUIRED)
  add_rostest_with_parameters_gmock(test_example_gmock test/example_test_gmock.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/example_params.yaml)
  target_include_directories(test_example_gmock PRIVATE include)
  target_link_libraries(test_example_gmock admittance_controller_parameters rclcpp::rclcpp)
endif()

ament_package()
