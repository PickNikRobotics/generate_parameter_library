find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(span_tests span_tests.cpp)
target_link_libraries(span_tests tcb_span)
