cmake_minimum_required(VERSION 3.5)
project(gen_param_struct_example)

# find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gen_param_struct REQUIRED)

add_executable(test_node main.cpp)

target_include_directories(test_node PRIVATE include)
ament_target_dependencies(test_node
        rclcpp
        gen_param_struct)

# specify which yaml root name to convert to struct
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/include/config/data.yaml)
# specify which yaml root name to convert to struct
set(YAML_TARGET admittance_controller)
# specify which yaml root name to convert to struct
set(OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/config)

generate_param_struct_header(test_node ${OUT_DIR} ${YAML_FILE} ${YAML_TARGET})

install(
        TARGETS test_node
        RUNTIME DESTINATION bin
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)


ament_package()