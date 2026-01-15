# generate_parameter_library_py_codegen.cmake
#
# Purpose
# -------
# Provide a reusable CMake function that downstream ROS2/ament packages can call
# to:
#   1) generate a Python module from a *.ros.schema file at build time, and
#   2) install the generated module into the correct ament Python install dir
#      (typically dist-packages for overlay workspaces on Ubuntu).
#
# This solves the dist-packages vs site-packages mismatch by delegating the
# install destination to ament (ament_get_python_install_dir), rather than
# hardcoding a Python path.
#
# Public API
# ----------
#   generate_parameter_library_generate_python_module(
#     TARGET <cmake_target_name>
#     PACKAGE <python_package_name>
#     MODULE  <module_basename>
#     SCHEMA  <path/to/file.ros.schema>
#     [VALIDATION_MODULE <python.import.path>]
#     [OUT_DIR <build_output_dir>]
#   )
#
# What this creates
# -----------------
# - Generated file: <OUT_DIR>/<MODULE>.py
# - Installed file: <prefix>/<ament_python_install_dir>/<PACKAGE>/<MODULE>.py
#
# Notes / assumptions to check in your environment
# ------------------------------------------------
# - The generator is imported as:
#     generate_parameter_library_py.generate_python_module.run
#   This assumes the python package `generate_parameter_library_py` is installed
#   and importable by the interpreter found by `find_package(Python3 ...)`.
#   In your container this is typically satisfied by the apt package:
#     ros-humble-generate-parameter-library-py
# - If the upstream python generator changes its API (run signature), this CMake
#   module must be updated accordingly. (A future enhancement is to add a stable
#   CLI entrypoint in the python package and call that instead of -c import/run.)
#
# Implementation detail
# ---------------------
# We intentionally generate into:
#   <build>/<consumer_pkg>/generated/<python_pkg>
# rather than:
#   <build>/<consumer_pkg>/<python_pkg>
# to avoid the common "build/<pkg>/<pkg>" import shadowing issues during tests.

function(generate_parameter_library_generate_python_module)
  # Parse named arguments into variables:
  #   ARG_TARGET, ARG_PACKAGE, ARG_MODULE, ARG_SCHEMA, ARG_VALIDATION_MODULE, ARG_OUT_DIR
  cmake_parse_arguments(
    ARG
    ""  # boolean options (none)
    "TARGET;PACKAGE;MODULE;SCHEMA;VALIDATION_MODULE;OUT_DIR"  # single-value args
    ""  # multi-value args (none)
    ${ARGN}
  )

  # Validate required arguments early so failures are clear at configure-time.
  foreach(req TARGET PACKAGE MODULE SCHEMA)
    if(NOT ARG_${req})
      message(FATAL_ERROR
        "generate_parameter_library_generate_python_module: ${req} is required")
    endif()
  endforeach()

  # We need a Python interpreter to run the generator.
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # We need ament_cmake_python so we can ask ament where Python packages
  # should be installed in this environment (dist-packages vs site-packages).
  #
  # Potential uncertainty:
  # - If someone uses this outside an ament environment, this will fail. In your
  #   ROS2 container this is expected and correct.
  find_package(ament_cmake_python REQUIRED)

  # This returns a path relative to CMAKE_INSTALL_PREFIX, typically like:
  #   lib/python3.10/dist-packages
  ament_get_python_install_dir(_PY_INSTALL_DIR)

  # Default output directory:
  # - Keep generated code in a build subdir that does NOT create build/<pkg>/<pkg>
  #   to avoid import shadowing problems in tests.
  if(NOT ARG_OUT_DIR)
    set(ARG_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/${ARG_PACKAGE}")
  endif()

  # Final output file path.
  set(_OUT_FILE "${ARG_OUT_DIR}/${ARG_MODULE}.py")

  # Optional validation module passed through to generator as a string.
  if(NOT ARG_VALIDATION_MODULE)
    set(ARG_VALIDATION_MODULE "")
  endif()

  # Define how to create the generated file.
  #
  # We use Python3_EXECUTABLE -c instead of a wrapper script so that downstream
  # packages do not need to carry their own generator scripts.
  #
  # Potential uncertainty:
  # - If the generator internally depends on files that change (not just SCHEMA),
  #   they are not listed in DEPENDS here because, in your apt distribution model,
  #   the generator is not expected to change during a workspace build. If you
  #   frequently develop the generator in-source, you can add extra DEPENDS on
  #   relevant python files.
  add_custom_command(
    OUTPUT "${_OUT_FILE}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${ARG_OUT_DIR}"
    COMMAND ${Python3_EXECUTABLE} -c
      "from generate_parameter_library_py.generate_python_module import run; run(r'${_OUT_FILE}', r'${ARG_SCHEMA}', r'${ARG_VALIDATION_MODULE}')"
    DEPENDS "${ARG_SCHEMA}"
    VERBATIM
  )

  # Ensure the custom command runs during a normal build (ALL).
  add_custom_target(${ARG_TARGET} ALL
    DEPENDS "${_OUT_FILE}"
  )

  # Install the generated file into the downstream python package directory
  # under the ament python install dir (dist-packages in your overlay).
  install(FILES "${_OUT_FILE}"
    DESTINATION "${_PY_INSTALL_DIR}/${ARG_PACKAGE}"
  )

  # Optional debug logging (leave commented for production):
  # message(STATUS "[generate_parameter_library] SCHEMA=${ARG_SCHEMA}")
  # message(STATUS "[generate_parameter_library] OUT_FILE=${_OUT_FILE}")
  # message(STATUS "[generate_parameter_library] INSTALL=${_PY_INSTALL_DIR}/${ARG_PACKAGE}")
endfunction()
