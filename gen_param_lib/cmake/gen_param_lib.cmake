function(gen_param_lib LIB_NAME YAML_FILE)
  find_program(gen_param_lib_BIN NAMES "gen_param_lib")
  if(NOT gen_param_lib_BIN)
    message(FATAL_ERROR "gen_param_lib() variable 'gen_param_lib_BIN' must not be empty")
  endif()

  # Optional 4th parameter for the user defined validation header
  set(VALIDATE_HEADER ${ARGV2})

  # Set the output parameter header file name
  set(PARAM_HEADER_FILE ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}/include/${LIB_NAME}.hpp)

  add_custom_command(
    OUTPUT ${PARAM_HEADER_FILE}
    COMMAND ${gen_param_lib_BIN} ${PARAM_HEADER_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE} ${VALIDATE_HEADER}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE}
    COMMENT
    "Running `${gen_param_lib_BIN} ${PARAM_HEADER_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE} ${VALIDATE_HEADER}`"
    VERBATIM
  )

  add_library(${LIB_NAME} ${PARAM_HEADER_FILE})
  target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}/include)
  set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${LIB_NAME}
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
  )
endfunction()
