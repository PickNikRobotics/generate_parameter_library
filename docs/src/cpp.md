# C++ usage

The generated header file is named based on the target library name you passed as the first argument to the cmake function.
If you specified it to be `turtlesim_parameters` you can then include the generated code with `#include <turtlesim/turtlesim_parameters.hpp>`.
```c++
#include <turtlesim/turtlesim_parameters.hpp>
```

Note that this header can also be used from another package:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_other_package)


include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(turtlesim REQUIRED)

add_library(my_lib src/my_lib.cpp)
target_include_directories(my_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(my_lib PUBLIC turtlesim::turtlesim_parameters)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

install(TARGETS my_lib
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(turtlesim)
ament_package()
```

In your initialization code, create a `ParamListener` which will declare and get the parameters.
An exception will be thrown if any validation fails or any required parameters were not set.
Then call `get_params` on the listener to get a copy of the `Params` struct.
```c++
auto param_listener = std::make_shared<turtlesim::ParamListener>(node);
auto params = param_listener->get_params();
```

## Dynamic parameters

If you are using dynamic parameters, you can use the following code to check if any of your parameters have changed and then get a new copy of the `Params` struct.
```c++
if (param_listener->is_old(params_)) {
  params_ = param_listener->get_params();
}
```

Alternatively, you can bind a callback function that triggers whenever a parameter is updated. When activated, the callback receives the updated parameters as an argument.
```c++
parameter_listener.setUserCallback([this](const auto& params) { reconfigure_callback(params); });
```

## Generated code

The YAML root names the C++ namespace. The generated `Params` struct holds parameter values, including nested structs and maps. `ParamListener` declares parameters, validates updates, and provides snapshots through `get_params()`. Keep the listener alive for as long as the node needs parameter updates.
