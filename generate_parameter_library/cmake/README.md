# CMake helpers

## Python code generation helper

This fork exports an additional CMake function:

`generate_parameter_library_generate_python_module(...)`

### Why this exists (fork-specific)

In some ROS 2 overlay/CI environments, Python modules generated at build time can
end up installed under a different Python directory than the one used at runtime
(e.g., `site-packages` vs `dist-packages`). This helper installs the generated
module into the Python install directory selected by ament via
`ament_get_python_install_dir()`, so downstream imports are consistent.

### Function signature

```cmake
generate_parameter_library_generate_python_module(
  TARGET <cmake_target_name>
  PACKAGE <python_package_name>
  MODULE  <module_basename>
  SCHEMA  <path/to/file.ros.schema>
  [VALIDATION_MODULE <python.import.path>]
  [OUT_DIR <build_output_dir>]
)
```

### Behavior

* Generates: `<OUT_DIR>/<MODULE>.py`
* Installs: `<prefix>/<ament_python_install_dir>/<PACKAGE>/<MODULE>.py`

If `OUT_DIR` is not provided, the default is:

`<build>/<consumer_pkg>/generated/<python_package_name>`

### Example

```cmake
find_package(generate_parameter_library REQUIRED)

generate_parameter_library_generate_python_module(
  TARGET  my_pkg__params_py
  PACKAGE my_pkg
  MODULE  params
  SCHEMA  ${CMAKE_CURRENT_SOURCE_DIR}/schema/params.ros.schema
)
```

### Notes

* The generator is invoked as:
  `generate_parameter_library_py.generate_python_module.run`.
* The generator must be importable by the interpreter found by:
  `find_package(Python3 ... COMPONENTS Interpreter)`.
