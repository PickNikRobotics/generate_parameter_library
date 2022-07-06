cmake_minimum_required(VERSION 3.5)
project(gen_param_struct)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
        DIRECTORY scripts/
        DESTINATION share/gen_param_struct/cmake
        USE_SOURCE_PERMISSIONS
)

install(
    DIRECTORY include/
    DESTINATION include
)


ament_export_include_directories(
    include
)

ament_package(CONFIG_EXTRAS
        gen_param_struct.cmake
        )
