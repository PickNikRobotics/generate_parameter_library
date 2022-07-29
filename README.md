# generate_parameter_library
Generate C++ for ROS 2 parameter declaration, getting, and validation using delcaritive YAML.
The generated library contains a C++ struct with specified parameters.
Additionally, dynamic parameters and custom validation are made easy.

## Killer Features
* Declarative YAML syntax for ROS 2 Parameters converted into C++ struct
* Declaring, Getting, Validating, and Updating handled by generated code
* Dynamic ROS 2 Parameters made easy
* Custom user specified validator functions

## Basic Usage
1. [Create YAML parameter codegen file](#create-yaml-parameter-codegen-file)
2. [Add parameter library generation to project](#add-parameter-library-generation-to-project)
3. [Use generated struct into project source code](#use-generated-struct-into-project-source-code)

### Create yaml parameter codegen file
Write a yaml file to declare your parameters and their attributes.

```yaml
turtlesim:
  background:
    r: {
      type: int,
      default_value: 0,
      description: "Red color value for the background, 8-bit",
      validation: {
        bounds: [0, 255]
      }
    }
    g: {
      type: int,
      default_value: 0,
      description: "Green color value for the background, 8-bit",
      validation: {
        bounds: [0, 255]
      }
    }
    b: {
      type: int,
      default_value: 0,
      description: "Blue color value for the background, 8-bit",
      validation: {
        bounds: [0, 255]
      }
    }
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

add_executable(turtlesim src/turtlesim.cpp)
target_link_libraries(minimal_node PRIVATE
  rclcpp::rclcpp
  turtlesim_parameters
)
```

### Use generated struct into project source code
```c++
#include <rclcpp/rclcpp.hpp>
#include "turtlesim_parameters.hpp"

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

## Detailed Documentation
* [Cpp namespace](#cpp-namespace)
* [Parameter definition](#parameter-definition)
* [Built-In Validators](#built-in-validators)
* [Custom validator functions](#custom-validator-functions)
* [Nested structures](#nested-structures)
* [Use generated struct in Cpp](#use-generated-struct-in-cpp)
* [Dynamic Parameters](#dynamic-parameters)
* [Example Project](#example-project)
* [Generated code output](#generated-code-output)

### Cpp namespace
The root element of the YAML file determines the namespace used in the generated C++ code.
We use this to put the `Params` struct in the same namespace as your C++ code.

```yaml
cpp_namespace:
# additionally fields  ...
```

### Parameter definition
The YAML syntax can be thought of as a tree since it allows for arbitrary nesting of key-value pairs.
For clarity, the last non-nested value is referred to as a leaf.
A leaf represents a single parameter and has the following format.

```yaml
cpp_namespace:
  param_name: {
    type: int,
    default_value: 3,
    read_only: true,
    description: "A read only  integer parameter with a default value of 3",
    validation:
      # validation functions ...
  }
```

A parameter is a YAML dictionary with the only required key being `type`.

| Field         | Description                                                   |
|---------------|---------------------------------------------------------------|
| type          | The type (string, double, etc) of the parameter.              |
| default_value | Value for the parameter if the user does not specify a value. |
| read_only     | Can only be set at launch and are not dynamic.                |
| description   | Displayed by `ros2 param describe`.                           |
| validation    | Dictionary of validation functions and their parameters.      |

The types of parameters in ros2 map to C++ types.

| Parameter Type  | C++ Type                    |
|-----------------|-----------------------------|
| string          | `std::string`               |
| double          | `double`                    |
| int             | `int`                       |
| bool            | `bool`                      |
| string_array    | `std::vector<std::string>`  |
| double_array    | `std::vector<double>`       |
| int_array       | `std::vector<int>`          |
| bool_array      | `std::vector<bool>`         |
| string_fixed_XX | `FixedSizeString<XX>`       |

Fixed size types are denoted with a suffix `_fixed_XX`, where `XX` is the desired size.
The corresponding C++ type is a data wrapper class for conveniently accessing the data.
Note that any fixed size type will automatically use a `size_lt` validator. Validators are explained in the next section.

### Built-In Validators
Validators are C++ functions that take arguments represented by a key-value pair in yaml.
The key is the name of the function.
The value is an array of values that are passed in as parameters to the function.
If the function does not take any values you write `null` or `[]` to for the value.

```yaml
joint_trajectory_controller:
  command_interfaces: {
    type: string_array,
    description: "Names of command interfaces to claim",
    validation: {
      size_gt<>: [0],
      unique<>: null,
      subset_of<>: [["position", "velocity", "acceleration", "effort",]],
    }
  }
```

Above are validations for `command_interfaces` from `ros2_controllers`.
This will require this string_array to have these properties:

* There is at least one value in the array
* All values are unique
* Values are only in the set `["position", "velocity", "acceleration", "effort",]`

You will note that some validators have a suffix of `<>`, this tells the code generator to pass the C++ type of the parameter as a function template.
The built-in validator functions provided by this package are:

| Function               | Arguments           | Description                                                           |
|------------------------|---------------------|-----------------------------------------------------------------------|
| unique<>               | []                  | Array type parameter contains no duplicates                           |
| subset_of<>            | [[val1, val2, ...]] | Every element of array type parameter is contained within argument    |
| fixed_size<>           | [length]            | Length of array or string is specified length                         |
| size_gt<>              | [length]            | Length of array or string is greater than specified length            |
| size_lt<>              | [length]            | Length of array or string is less less specified length               |
| not_empty<>            | []                  | Array or string parameter is not empty                                |
| element_bounds<>       | [lower, upper]      | Bounds checking for every element of array type parameter (inclusive) |
| lower_element_bounds<> | [lower]             | Lower bound for every element of array type parameter (inclusive)     |
| upper_element_bounds<> | [upper]             | Upper bound for every element of array type parameter (inclusive)     |
| bounds<>               | [lower, upper]      | Bounds checking for a scalar type parameter (inclusive)               |
| lower_bounds<>         | [lower]             | Lower bounds for a scalar type parameter (inclusive)                  |
| upper_bounds<>         | [upper]             | Upper bounds for a scalar type parameter (inclusive)                  |
| one_of<>               | [[val1, val2, ...]] | Scalar type parameter is one of the specified values                  |

### Custom validator functions
Validators are functions that return a `Result` type and accept a `rclcpp::Parameter const&` as their first argument and any number of arguments after that can be specified in YAML.
Validators are C++ functions defined in a header file similar to the example shown below.

The `Result` type has a alias `OK` that is shorthand for returning a successful validation.
It also had a function `ERROR` that uses the expressive [fmt format](https://github.com/fmtlib/fmt) for constructing a human readable error.
These come from the `parameter_traits` library.
Note that you need to place your custom validators in the `parameter_traits` namespace.

```c++
#include <parameter_traits/parameter_traits.hpp>

namespace parameter_traits {

Result integer_equal_value(rclcpp::Parameter const& parameter, int expected_value) {
  int param_value = parameter.as_int();
    if (param_value != expected_value) {
        return ERROR("Invalid value {} for parameter {}. Expected {}",
               param_value, parameter.get_name(), expected_value);
    }

  return OK;
}

}  // namespace parameter_traits
```
To configure a parameter to be validated with the custom validator function `integer_equal_value` with an `expected_value` of `3` you could would this to the YAML.
```yaml
validation: {
  integer_equal_value: [3]
}
```

### Nested structures
After the top level key, every subsequent non-leaf key will generate a nested c++ struct. The struct instance will have
the same name as the key.

```yaml
cpp_name_space:
  nest1:
    nest2:
      param_name: { # this is a leaf
        type: string_array
      }
```

The generated parameter value can then be access with `params.nest1.nest2.param_name`

### Use generated struct in Cpp
The generated header file is named based on the target library name you passed as the first argument to the cmake function.
If you specified it to be `turtlesim_parameters` you can then include the generated code with `#include "turtlesim_parameters.hpp"`.
```c++
#include "turtlesim_parameters.hpp"
```
In your initialization code, create a `ParamListener` which will declare and get the parameters.
An exception will be thrown if any validation fails or any required parameters were not set.
Then call `get_params` on the listener to get a copy of the `Params` struct.
```c++
auto param_listener = std::make_shared<turtlesim::ParamListener>(node);
auto params = param_listener->get_params();
```

### Dynamic Parameters
If you are using dynamic parameters, you can use the following code to check if any of your parameters have changed and then get a new copy of the `Params` struct.
```c++
if (param_listener->is_old(params_)) {
  params_ = param_listener->get_params();
}
```

### Example Project
See [example project](example/) for a complete example of how to use the generate_parameter_library.

### Generated code output
The generated code is primarily consists of two major components:
1) `struct Params` that contains values of all parameters and
2) `class ParamListener` that handles parameter declaration, updating, and validation.
   The general structure is shown below.

```cpp
namespace cpp_namespace {

struct Params {
  int param_name = 3;
  struct {
    struct{
      std::string param_name;
      // arbitrary nesting depth...
    } nest2;
  } nest1;
  // for detecting if the parameter struct has been updated
  rclcpp::Time __stamp;
};

class ParamListener {
 public:
  ParamListener(rclcpp::ParameterInterface);
  ParamListener(rclcpp::Node::SharedPtr node)
    : ParameterListener(node->get_parameters_interface()) {}
  ParamListener(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : ParameterListener(node->get_parameters_interface()) {}

  // create a copy of current parameter values
  Params get_params() const;

  // returns true if parameters have been updated since last time get_params was called
  bool is_old(Params const& other) const;

  // loop over all parameters: perform validation then update
  rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters);

  // declare all parameters and throw exception if non-optional value is missing or validation fails
  void declare_params(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>& parameters_interface);

 private:
  Params params_;
};

} // namespace cpp_namespace
```
The structure of the `Params` struct and the logic for declaring and updating parameters is generated from a YAML file specification.
