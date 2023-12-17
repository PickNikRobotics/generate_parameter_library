cmake_minimum_required(VERSION 3.16)
project(cmake_generate_parameter_module_example)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_module(admittance_parameters
  cmake_generate_parameter_module_example/parameters.yaml
  cmake_generate_parameter_module_example.custom_validation
)

ament_python_install_package(${PROJECT_NAME})

ament_package()
