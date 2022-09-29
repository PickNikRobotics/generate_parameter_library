cmake_minimum_required(VERSION 3.16)
project(parameter_traits)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tcb_span REQUIRED)

add_library(parameter_traits INTERFACE)
target_include_directories(parameter_traits INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(parameter_traits INTERFACE cxx_std_17)
target_link_libraries(parameter_traits
  INTERFACE
    fmt::fmt
    rclcpp::rclcpp
    tcb_span::tcb_span
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linters
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS parameter_traits
  EXPORT export_parameter_traits
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_parameter_traits HAS_LIBRARY_TARGET)
ament_export_dependencies(fmt rclcpp tcb_span)
ament_package()
