cmake_minimum_required(VERSION 3.22)
project(tcb_span)

find_package(ament_cmake REQUIRED)

add_library(tcb_span INTERFACE)
target_include_directories(tcb_span INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_compile_features(tcb_span INTERFACE cxx_std_17)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS tcb_span
  EXPORT export_tcb_span
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_tcb_span)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following lines skip linters
  set(ament_cmake_cpplint_FOUND TRUE) # Conflicts with clang-foramt
  set(ament_cmake_uncrustify_FOUND TRUE) # Conflicts with clang-format
  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)
endif()

ament_package()
