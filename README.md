# generate_parameter_library

This package aims to automate parameter handling within ROS 2 via c++ code generation. A user can specify parameter
names, types, descriptions, and validation functions all within the YAML syntax. This packages provides a
straightforward interface to integrate the code generation into cmake builds. The generated library contains a c++
struct with all specified parameters as nested fields. Additionally, methods are generated, which enable dynamic parameter
updating, parameter copying, and parameter validation.

## Generated code interface
The generated code is primarily consists of two major component: 1) a pure data struct `struct Params` that contains values of all parameters and 2) a class `ParamListener` that handles parameter declaration, updating, and validation. The general structure is shown below.

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

## Usage

Integrating the generate_parameter_library into a project requires the following items:
<ol>
  <li>Create YAML parameter codegen file</li>
  <li>Add parameter library generation to cmake project</li>
  <li>Integrate generated struct into project source code</li>
</ol>

Throughout the instructions, the following minimal sample project will be referred to:

```c++
#include "rclcpp/rclcpp.hpp"
// additional includes

class MinimalInterface {
public:
    MinimalInterface(rclcpp::Node::SharedPtr node) {
    // Initialization ...
    }
    void update(const std_msgs::msg::String::SharedPtr msg) {
    // Update state  ...
    }
    // Member variables ...
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("minimal_node");
  // Initialization ...
  while(rclcpp::ok()){
    // do update
  }

  return 0;
}

```
```cmake
cmake_minimum_required(VERSION 3.22)
project(minimal_project)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# additional find_package ...
add_executable(minimal_node
  src/main.cpp
)
# additional libraries ...
install(
  TARGETS test_node
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

```

The sample program has two primary stages. The initialization stage runs only once at the beginning of the program. The
update stage runs an arbitrary number times until the program exits.

## Create yaml parameter codegen file

YAML should be a familiar language to ROS developers as it is used to specify parameter
configurations. It also has a minimal and human-readable syntax, which make it a viable candidate for specifying code
generation. The code generation syntax used by this package is defined in the following subsections.

### Name space specification

The generated c++ code contains a struct with type `Params` and a class of type `ParamListener`. Each are generated within specified name space. YAML allows key value pairs to be defined in a hierarchical way. The top level key will be used as the name space.

```yaml
cpp_name_space:
# additionally fields  ...
```

### Parameter type definition

The YAML syntax can be thought of as a tree since it allows for arbitrary nesting of key value pairs. For clarity, the
last non-nested value is referred to as a leaf. A leaf has the following required format.

```yaml
cpp_name_space:
  param_definition: { # this is a leaf
    type: string_array
  }
```

The leaf is a YAML dictionary with the only required key being type. The value of type can be any ROS 2 parameter
type, e.g. string, string_array, double, etc. Several non-required key value pairs are supported as shown below.

```yaml
cpp_name_space:
  param_name: { # this is a leaf
    type: int,
    default_value: 3,
    read_only: true,
    description: "A read only  integer parameter with a default value of 3",
    validation:
      # validation functions ...
  }
```

<ol>
  <li>The `default_value` sets the parameter's value if a value does not already exist.</li>
  <li>The `read_only` specifies whether a parameter can be changed via the ROS2 API.</li>
  <li>The `description` specifies the parameter description, which is displayed with `ros2 param describe` tool.</li>
</ol>

### Validators specification

Parameters are run through validators during both initialization and updating. If a validation fails during initialization,
an exception will be thrown. If a validation fails during updating, the parameter will simply not be updated and a
failure message will be returned via the ROS 2 API. Validators are c++ functions defined in a header file with the
following interface.

```c++
Result integer_equal_value(rclcpp::Parameter const& parameter, int expected_value) {
  int param_value = parameter.as_int();
    if (param_value != expected_value) {
        return ERROR("Invalid value {} for parameter {}. Expected {}",
               param_value, parameter.get_name(), expected_value);
    }

  return OK;
}
```
The YAML syntax that adds the above validator to the generated code is shown below.
```yaml
validation: {
    integer_equal_value: [3]
  }
```
The `validation` is a YAML dictionary, where each key refers to a validation function and each value refers to the functions' input arguments. Validator arguments are specified with a YAML list. If a validation function is templated, then the suffix `<>` is added to the function name. For example, `integer_equal_value` would be changed to `integer_equal_value<>`. Alternatively, the template specialization can be explicitly written, e.g `integer_equal_value<int>`.


The following validation functions are provided by default in this package.

| Validation functions | Description                                                                  | Arguments           |
|----------------------|------------------------------------------------------------------------------|---------------------|
| unique               | Validates array type parameter contains no duplicates                        | [null]              |
| subset_of            | Validates every element of array type parameter is contained within argument | [[val1, val2, ...]] |
| fixed_size           | Validates array type parameter is of specified length                        | [length]            |
| size_gt              | Validates array type parameter is greater than specified length              | [length]            |
| size_lt              | Validates array type parameter is greater less specified length              | [length]            |
| element_bounds       | Validates every element of array type parameter is in bounds (inclusive)     | [[lower, upper]]    |
| lower_element_bounds | Validates lower bound for every element of array type parameter              | [lower]             |
| upper_element_bounds | Validates upper bound for every element of array type parameter              | [upper]             |
| bounds               | Validates scalar type parameter is in bounds (inclusive)                     | [[lower, upper]]    |
| lower_bounds         | Validates lower bounds for a scalar type parameter                           | [lower]             |
| upper_bounds         | Validates upper bounds for a scalar type parameter                           | [upper]             |
| one_of               | Validates scalar type parameter is one of the specified values               | [val1, val2, ...]   |

### Nested structure syntax

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

### Complete YAML file
The complete sample file is shown below. By convention, this file should be named the same as the node that uses it with a `_parameter` suffix: `{node_name}_parameters.yaml`.
```yaml
cpp_name_space:
  param_name: { # this is a leaf
    type: int,
    default_value: 3,
    read_only: true,
    description: "A read only  integer parameter with a default value of 3",
    validation: {
      integer_equal_value: [3]
    }
  }
  nest1:
    nest2:
      param_name: { # this is a leaf
        type: string_array
      }
```

## Add parameter library generation to cmake project

To utilize this package, you must include it in a cmake project with `find_package`. Doing so will add a cmake macro called `generate_parameter_library`, which is used to generate the parameter library. In the cmake example, the following should be substituted for `# additional find_package ...`
```cmake
find_package(generate_parameter_library REQUIRED)
```

After adding the executable, which will depend on the generated code, `generate_parameter_library` is called with the following arguments.

``` cmake
generate_parameter_library(
minimal_node_parameters # cmake target name for the parameter library
src/minimal_node_parameters.yaml # path to input yaml file
include/minimal_node/validators.hpp # Optional path to header containing custom validators
)
```
The generated library then needs to be linked to the executable.
```cmake
target_link_libraries(minimal_node PRIVATE
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  minimal_node_parameters
)
```
The above cmake is substituted in place of `# additional libraries ...`. Notably, the third argument of `generate_parameter_library` specifies a user provided header to include in the generated code. All validation functions defined in this header can be referred to in the YAML validation dictionary.

## Integrate generated struct into project source code
Only a few source code changes are needed to integrate the generated parameter library code. First, the library must be included via `#include "minimal_node_parameters.hpp"`. Note that the include name is exactly the same as the generated library name specified for in cmake.
```c++
#include "minimal_node_parameters.hpp"
```
In the initialization code, an instance of `ParamListener` and `Params` should be created. At this time, parameters will be declared and read. An exception will be thrown if any validation fails or any required parameters were not set.
```c++
param_listener = std::make_shared<minimal_node::ParamListener>(node);
params_ = param_listener->get_params();
```
Finally, the instance of `Params` can be updated dynamically during the main loop as follows.
```c++
if (param_listener->is_old(params_)) {
  params_ = param_listener->get_params();
}
```
Substituting in all changes, the new source is shown below.
```c++
#include "rclcpp/rclcpp.hpp"
#include "minimal_node_parameters.hpp"

class MinimalInterface {
public:
    MinimalInterface(rclcpp::Node::SharedPtr node) {
        param_listener = std::make_shared<minimal_node::ParamListener>(node);
        params_ = param_listener->get_params();
    }
    void update(const std_msgs::msg::String::SharedPtr msg) {
        if (param_listener->is_old(params_)) {
          params_ = param_listener->get_params();
        }
      // ...
    }

    std::share_ptr<minimal_node::ParamListener> param_listener;
    minimal_node::Params params_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("minimal_node");
  auto minimal_interface = MinimalInterface(node);

  while(rclcpp::ok()){
    minimal_interface.update();
    // ...
  }

  return 0;
}

```

See [example project](example/) for an example of how to use the generate_parameter_library.
