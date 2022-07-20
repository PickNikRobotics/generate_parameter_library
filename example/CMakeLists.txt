cmake_minimum_required(VERSION 3.22)
project(generate_parameter_library_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(admittance_controller_parameters
  src/parameters.yaml
  include/generate_parameter_library_example/validators.hpp
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

ament_package()
