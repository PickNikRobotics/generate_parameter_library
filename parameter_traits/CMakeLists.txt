cmake_minimum_required(VERSION 3.16)
project(parameter_traits)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rsl REQUIRED)
find_package(tcb_span REQUIRED)
find_package(tl_expected REQUIRED)

add_library(parameter_traits INTERFACE)
target_include_directories(parameter_traits INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/parameter_traits>)
target_compile_features(parameter_traits INTERFACE cxx_std_17)
target_link_libraries(parameter_traits
  INTERFACE
    fmt::fmt
    rclcpp::rclcpp
    rsl::rsl
    tcb_span::tcb_span
    tl_expected::tl_expected
)

install(
  DIRECTORY include/
  DESTINATION include/parameter_traits
)

install(
  TARGETS parameter_traits
  EXPORT export_parameter_traits
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_parameter_traits HAS_LIBRARY_TARGET)
ament_export_dependencies(fmt rclcpp rsl tcb_span tl_expected)

find_package(hydra_cpack REQUIRED)

ament_package()

hydra_cpack_ros(VERSION ${HYDRA_CPACK_VERSION})
