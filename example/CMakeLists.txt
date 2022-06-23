

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# specify which yaml root name to convert to struct
set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/data.yaml)
# specify which yaml root name to convert to struct
set(YAML_TARGET admittance_controller)
# specify which yaml root name to convert to struct
set(OUT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/parameters)


generate_param_struct_header(${OUT_DIR} ${YAML_FILE} ${YAML_TARGET})

add_executable(test_node main.cpp)
add_dependencies(test_node run)
target_include_directories(test_node PRIVATE include)
ament_target_dependencies(test_node
        rclcpp
        std_msgs)


install(TARGETS test_node DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/install)


