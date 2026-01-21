# generate_parameter_library_py_codegen.cmake
#
# Downstream helper to generate and install a Python module from a *.ros.schema.
# The install destination is resolved via ament (dist-packages vs site-packages).

function(generate_parameter_library_generate_python_module)
  cmake_parse_arguments(
    ARG
    ""
    "TARGET;PACKAGE;MODULE;SCHEMA;VALIDATION_MODULE;OUT_DIR"
    ""
    ${ARGN}
  )

  foreach(req TARGET PACKAGE MODULE SCHEMA)
    if(NOT ARG_${req})
      message(
        FATAL_ERROR
        "generate_parameter_library_generate_python_module: ${req} is required"
      )
    endif()
  endforeach()

  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_python REQUIRED)

  ament_get_python_install_dir(_py_install_dir)

  if(NOT ARG_OUT_DIR)
    set(ARG_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/${ARG_PACKAGE}")
  endif()

  set(_out_file "${ARG_OUT_DIR}/${ARG_MODULE}.py")

  if(NOT ARG_VALIDATION_MODULE)
    set(ARG_VALIDATION_MODULE "")
  endif()

  add_custom_command(
    OUTPUT "${_out_file}"
    BYPRODUCTS "${_out_file}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${ARG_OUT_DIR}"
    COMMAND ${Python3_EXECUTABLE} -c
      "from generate_parameter_library_py.generate_python_module import run; \
run(r'${_out_file}', r'${ARG_SCHEMA}', r'${ARG_VALIDATION_MODULE}')"
    DEPENDS "${ARG_SCHEMA}"
    COMMENT "Generating ${ARG_PACKAGE}/${ARG_MODULE}.py from ${ARG_SCHEMA}"
    VERBATIM
  )

  add_custom_target(${ARG_TARGET} ALL
    DEPENDS "${_out_file}"
  )

  install(
    FILES "${_out_file}"
    DESTINATION "${_py_install_dir}/${ARG_PACKAGE}"
  )
endfunction()
