# Getting started

Start with a ROS 2 workspace and the `generate_parameter_library` dependency installed. The YAML schema describes parameters; ROS parameter override files supply values at runtime.

1. [Create YAML parameter codegen file](#create-yaml-parameter-codegen-file)
2. [Add parameter library generation to project](#add-parameter-library-generation-to-project)
3. [Use generated struct in project source code](#use-generated-struct-in-project-source-code)

## Create yaml parameter codegen file
Write a yaml file to declare your parameters and their attributes.

**src/turtlesim_parameters.yaml**
```yaml
turtlesim:
  background:
    r:
      type: int
      default_value: 0
      description: "Red color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    g:
      type: int
      default_value: 0
      description: "Green color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
    b:
      type: int
      default_value: 0
      description: "Blue color value for the background, 8-bit"
      validation:
        bounds<>: [0, 255]
```

## Add parameter library generation to project

**package.xml**
```xml
<depend>generate_parameter_library</depend>
```

**CMakeLists.txt**
```cmake
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(
  turtlesim_parameters # cmake target name for the parameter library
  src/turtlesim_parameters.yaml # path to input yaml file
)

add_executable(minimal_node src/turtlesim.cpp)
target_link_libraries(minimal_node PRIVATE
  rclcpp::rclcpp
  turtlesim_parameters
)

install(TARGETS minimal_node turtlesim_parameters
  EXPORT ${PROJECT_NAME}Targets)
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
```

**setup.py**
```python
from generate_parameter_library_py.setup_helper import generate_parameter_module

generate_parameter_module(
  "turtlesim_parameters", # python module name for parameter library
  "turtlesim/turtlesim_parameters.yaml", # path to input yaml file
)
```

## Use generated struct in project source code

**src/turtlesim.cpp**
```c++
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/turtlesim_parameters.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("turtlesim");
  auto param_listener = std::make_shared<turtlesim::ParamListener>(node);
  auto params = param_listener->get_params();

  auto color = params.background;
  RCLCPP_INFO(node->get_logger(),
    "Background color (r,g,b): %d, %d, %d",
    color.r, color.g, color.b);

  return 0;
}
```

**turtlesim/turtlesim.py**
```python
import rclpy
from rclpy.node import Node
from turtlesim import turtlesim_parameters

def main(args=None):
  rclpy.init(args=args)
  node = Node("turtlesim")
  param_listener = turtlesim_parameters.ParamListener(node)
  params = param_listener.get_params()

  color = params.background
  node.get_logger().info(
    "Background color (r,g,b): %d, %d, %d" %
    color.r, color.g, color.b)
```

## Use example yaml files in tests
When using parameter library generation it can happen that there are issues when executing tests since parameters are not defined and the library defines them as mandatory.
To overcome this it is recommended to define example yaml files for tests and use them as follows:

```cmake
find_package(ament_cmake_gtest REQUIRED)
add_rostest_with_parameters_gtest(test_turtlesim_parameters test/test_turtlesim_parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/example_turtlesim_parameters.yaml)
target_include_directories(test_turtlesim_parameters PRIVATE include)
target_link_libraries(test_turtlesim_parameters turtlesim_parameters)
ament_target_dependencies(test_turtlesim_parameters rclcpp)
```

when using `gtest`, or:

```cmake
find_package(ament_cmake_gmock REQUIRED)
add_rostest_with_parameters_gmock(test_turtlesim_parameters test/test_turtlesim_parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/example_turtlesim_parameters.yaml)
target_include_directories(test_turtlesim_parameters PRIVATE include)
target_link_libraries(test_turtlesim_parameters turtlesim_parameters)
ament_target_dependencies(test_turtlesim_parameters rclcpp)
```
when using `gmock` test library.

🤖 P.S. having this example yaml files will make your users very grateful because they will always have a working example of a configuration for your node.
