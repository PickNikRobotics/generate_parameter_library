cmake_minimum_required(VERSION 3.22)
project(gen_param_struct)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# specify which yaml root name to convert to struct
set(YAML_TARGET admittance_controller)

add_custom_target(
        run ALL
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/gen_param_struct.py ${CMAKE_CURRENT_SOURCE_DIR} data.yaml ${YAML_TARGET}
        BYPRODUCTS ${YAML_TARGET}.h
        COMMENT "Generating yaml struct for ${YAML_TARGET}"
)

add_executable(test_node main.cpp)
add_dependencies(test_node run)
target_include_directories(test_node PRIVATE include)
ament_target_dependencies(test_node
        rclcpp
        std_msgs)


install(TARGETS test_node DESTINATION ${CMAKE_INSTALL_PREFIX})
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/install)


