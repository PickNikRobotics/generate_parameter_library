# Copyright 2022 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


function(generate_parameter_library LIB_NAME YAML_FILE)
  find_program(generate_parameter_library_py_BIN NAMES "generate_parameter_library_py")
  if(NOT generate_parameter_library_py_BIN)
    message(FATAL_ERROR "generate_parameter_library_py() variable 'generate_parameter_library_py_BIN' must not be empty")
  endif()

  # Make the include directory
  set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/${LIB_NAME}/include/)
  file(MAKE_DIRECTORY ${LIB_INCLUDE_DIR})

  # Optional 3rd parameter for the user defined validation header
  if(${ARGC} EQUAL 3)
    cmake_path(SET IN_VALIDATE_HEADER ${CMAKE_CURRENT_SOURCE_DIR})
    cmake_path(APPEND IN_VALIDATE_HEADER ${ARGV2})

    cmake_path(GET IN_VALIDATE_HEADER FILENAME VALIDATE_HEADER_FILENAME)
    cmake_path(SET VALIDATE_HEADER ${LIB_INCLUDE_DIR})
    cmake_path(APPEND VALIDATE_HEADER ${VALIDATE_HEADER_FILENAME})

    # Copy the header file into the include directory
    add_custom_command(
      OUTPUT ${VALIDATE_HEADER}
      COMMAND ${CMAKE_COMMAND} -E copy ${IN_VALIDATE_HEADER} ${VALIDATE_HEADER}
      DEPENDS ${IN_VALIDATE_HEADER}
      COMMENT
      "Running `${CMAKE_COMMAND} -E copy ${IN_VALIDATE_HEADER} ${VALIDATE_HEADER}`"
      VERBATIM
    )
  endif()

  # Set the yaml file parameter to be relative to the current source dir
  set(YAML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE})

  # Set the output parameter header file name
  set(PARAM_HEADER_FILE ${LIB_INCLUDE_DIR}/${LIB_NAME}.hpp)

  # Generate the header for the library
  add_custom_command(
    OUTPUT ${PARAM_HEADER_FILE}
    COMMAND ${generate_parameter_library_py_BIN} ${PARAM_HEADER_FILE} ${YAML_FILE} ${VALIDATE_HEADER_FILENAME}
    DEPENDS ${YAML_FILE} ${VALIDATE_HEADER}
    COMMENT
    "Running `${generate_parameter_library_py_BIN} ${PARAM_HEADER_FILE} ${YAML_FILE} ${VALIDATE_HEADER_FILENAME}`"
    VERBATIM
  )

  # Create the library target
  add_library(${LIB_NAME} ${PARAM_HEADER_FILE} ${VALIDATE_HEADER})
  target_include_directories(${LIB_NAME} PUBLIC
    $<BUILD_INTERFACE:${LIB_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:include/>
  )
  set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(${LIB_NAME}
    fmt::fmt
    parameter_traits::parameter_traits
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    tcb_span::tcb_span
  )
  install(DIRECTORY ${LIB_INCLUDE_DIR} DESTINATION include/)
endfunction()
