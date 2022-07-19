cmake_minimum_required(VERSION 3.5)
project(gen_param_struct_example)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(gen_param_struct REQUIRED)

add_executable(test_node main.cpp)

target_include_directories(test_node PRIVATE include)
ament_target_dependencies(test_node
        rclcpp
    rclcpp_lifecycle
        gen_param_struct)

# specify which yaml root name to convert to struct
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/include/config/definition.yaml)
# specify which yaml root name to convert to struct
set(OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/config)

generate_param_struct_header(test_node ${OUT_DIR} ${YAML_FILE})

install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME})
  
install(
        DIRECTORY include/
        DESTINATION include
)


ament_package()
