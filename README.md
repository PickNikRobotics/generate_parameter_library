# generate_parameter_library
Generate C++ or Python code for ROS 2 parameter declaration, getting, and validation using declarative YAML.
The generated library contains a C++ struct with specified parameters.
Additionally, dynamic parameters and custom validation are made easy.

**[Documentation](https://picknikrobotics.github.io/generate_parameter_library/)** · [Validator reference](docs/src/validators/index.md) · [Array validators](docs/src/validators/arrays.md)

## TOC
- [Killer Features](#killer-features)
- [Basic Usage](#basic-usage)
- [Detailed Documentation](#detailed-documentation)
- [FAQ](#faq)
- [Build status](#build-status)

## Killer Features
* Declarative YAML syntax for ROS 2 Parameters converted into C++ or Python struct
* Declaring, Getting, Validating, and Updating handled by generated code
* Dynamic ROS 2 Parameters made easy
* Custom user-specified validator functions
* Automatically create documentation of parameters

## Basic Usage
1. [Create YAML parameter codegen file](#create-yaml-parameter-codegen-file)
2. [Add parameter library generation to project](#add-parameter-library-generation-to-project)
3. [Use generated struct in project source code](#use-generated-struct-in-project-source-code)

### Create yaml parameter codegen file
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

### Add parameter library generation to project

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

### Use generated struct in project source code

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

### Use example yaml files in tests
When using parameter library generation it can happen that there are issues when executing tests since parameters are not defined and the library defines them as mandatory.
To overcome this it is recommended to define example yaml files for tests and use them as follows:

```
find_package(ament_cmake_gtest REQUIRED)
add_rostest_with_parameters_gtest(test_turtlesim_parameters test/test_turtlesim_parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/example_turtlesim_parameters.yaml)
target_include_directories(test_turtlesim_parameters PRIVATE include)
target_link_libraries(test_turtlesim_parameters turtlesim_parameters)
ament_target_dependencies(test_turtlesim_parameters rclcpp)
```

when using `gtest`, or:

```
find_package(ament_cmake_gmock REQUIRED)
add_rostest_with_parameters_gmock(test_turtlesim_parameters test/test_turtlesim_parameters.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/test/example_turtlesim_parameters.yaml)
target_include_directories(test_turtlesim_parameters PRIVATE include)
target_link_libraries(test_turtlesim_parameters turtlesim_parameters)
ament_target_dependencies(test_turtlesim_parameters rclcpp)
```
when using `gmock` test library.

🤖 P.S. having this example yaml files will make your users very grateful because they will always have a working example of a configuration for your node.

## Detailed Documentation

Read the [searchable documentation](https://picknikrobotics.github.io/generate_parameter_library/) or browse the [book sources](docs/src/SUMMARY.md).

### Cpp namespace

See [Cpp namespace](docs/src/yaml-syntax.md).

### Parameter definition

See [Parameter definition](docs/src/yaml-syntax.md).

### Built-In Validators

See [Built-In Validators](docs/src/validators/index.md).

### Custom validator functions

See [Custom validator functions](docs/src/validators/custom.md).

### Nested structures

See [Nested structures](docs/src/yaml-syntax.md).

### Mapped parameters

See [Mapped parameters](docs/src/mapped-parameters.md).

#### Key array scope resolution

See [Key array scope resolution](docs/src/mapped-parameters.md#key-array-scope-resolution).

### Use generated struct in Cpp

See [Use generated struct in Cpp](docs/src/cpp.md).

### Dynamic Parameters

See [Dynamic Parameters](docs/src/cpp.md).

### Parameter documentation

See [Parameter documentation](docs/src/parameter-documentation.md).

### Example Project

See [Example Project](docs/src/examples.md).

### Generated code output

See [Generated code output](docs/src/cpp.md).

### Generate markdown documentation

See [Generate markdown documentation](docs/src/parameter-documentation.md).

# FAQ

See the [FAQ](docs/src/faq.md).

## Build status

ROS2 Distro | Branch | Build status | Documentation | Package Build
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`main`](https://github.com/PickNikRobotics/generate_parameter_library/tree/main) | [![Rolling Binary Build](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/rolling-binary-build.yaml/badge.svg?branch=main)](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/rolling-binary-build.yaml?branch=main) <br> [![Rolling Semi-Binary Build](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/rolling-semi-binary-build.yaml/badge.svg?branch=main)](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/rolling-semi-binary-build.yaml?branch=main) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__generate_parameter_library__ubuntu_resolute_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__generate_parameter_library__ubuntu_resolute_amd64/) | [Documentation](https://docs.ros.org/en/rolling/p/generate_parameter_library/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__generate_parameter_library__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__generate_parameter_library__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uR64__generate_parameter_library_py__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Rbin_uR64__generate_parameter_library_py__ubuntu_resolute_amd64__binary/)
**Lyrical** | [`main`](https://github.com/PickNikRobotics/generate_parameter_library/tree/main) | see above <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Ldev__generate_parameter_library__ubuntu_resolute_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Ldev__generate_parameter_library__ubuntu_resolute_amd64/) | [Documentation](https://docs.ros.org/en/rolling/p/generate_parameter_library/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__generate_parameter_library__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__generate_parameter_library__ubuntu_resolute_amd64__binary/) <br> [![Build Status](https://build.ros2.org/buildStatus/icon?job=Lbin_uR64__generate_parameter_library_py__ubuntu_resolute_amd64__binary)](https://build.ros2.org/job/Lbin_uR64__generate_parameter_library_py__ubuntu_resolute_amd64__binary/)
**Kilted** | [`humble`](https://github.com/PickNikRobotics/generate_parameter_library/tree/humble) | see below <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Kdev__generate_parameter_library__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Kdev__generate_parameter_library__ubuntu_noble_amd64/) | [Documentation](https://docs.ros.org/en/kilted/p/generate_parameter_library/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__generate_parameter_library__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__generate_parameter_library__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/buildStatus/icon?job=Kbin_uN64__generate_parameter_library_py__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Kbin_uN64__generate_parameter_library_py__ubuntu_noble_amd64__binary/)
**Jazzy** | [`humble`](https://github.com/PickNikRobotics/generate_parameter_library/tree/humble) | see below <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Jdev__generate_parameter_library__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Jdev__generate_parameter_library__ubuntu_noble_amd64/) | [Documentation](https://docs.ros.org/en/jazzy/p/generate_parameter_library/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__generate_parameter_library__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__generate_parameter_library__ubuntu_noble_amd64__binary/) <br> [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__generate_parameter_library_py__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__generate_parameter_library_py__ubuntu_noble_amd64__binary/)
**Humble** | [`humble`](https://github.com/PickNikRobotics/generate_parameter_library/tree/humble) | [![Humble Binary Build](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/humble-binary-build.yaml/badge.svg?branch=humble)](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/humble-binary-build.yaml?branch=humble) <br> [![Humble Semi-Binary Build](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/humble-semi-binary-build.yaml/badge.svg?branch=humble)](https://github.com/PickNikRobotics/generate_parameter_library/actions/workflows/humble-semi-binary-build.yaml?branch=humble) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Hdev__generate_parameter_library__ubuntu_jammy_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Hdev__generate_parameter_library__ubuntu_jammy_amd64/) | [Documentation](https://docs.ros.org/en/humble/p/generate_parameter_library/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__generate_parameter_library__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__generate_parameter_library__ubuntu_jammy_amd64__binary/) <br> [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__generate_parameter_library_py__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__generate_parameter_library_py__ubuntu_jammy_amd64__binary/)
