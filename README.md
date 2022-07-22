# generate_parameter_library

This package aims to automate parameter handling within ROS 2 via c++ code generation. A user can specify parameter
names, types, descriptions, and validation functions all within the yaml syntax. This packages provides a
straightforward interface to integrate the code generation into cmake builds. The generated library contains a c++
struct with all specified parameters as nested fields. Additional, methods are generated, which enable dynamic parameter
updating, parameter copying, and parameter validation.

# Usage

Integrating the generate_parameter_library into a project requires the following items
<ol>
  <li>Create yaml parameter codegen file</li>
  <li>Add parameter library generation to cmake project</li>
  <li>Integrate generated struct into project source code</li>
</ol>

Throughout the instructions, the following minimal sample project will be referred to:

```c++
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

YAML should be a familiar data-serialization language to ROS developers as it is used to specify parameter
configurations. In addition, it has a minimal human-readable syntax which make it a viable candidate for specifying code
generation instructions. The code generation syntax in defined in the following subsections.

### Name space

The generated c++ struct type and class called `ParamListener` will be defined with the specified name space. YAML
allows defines key value pairs in a hierarchical way. The top level key will be used as the name space.

```yaml
cpp_name_space:
# additionally fields  ...
```

### Parameter type definition

The YAML syntax can be thought of as a tree since it allows for arbitrary nesting of key value pairs. For clearity, the
last non-nested value is referred to as a leaf. A leaf has the following required format.

```yaml
cpp_name_space:
  param_definition: { # this is a leaf 
    type: string_array
  }
```

The leaf is a YAML dictionary with the only required key value being type. The value of type can be any ROS 2 parameter
type, e.g. string, string_array, double, etc. Several non-required key value are supported as shown below.

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
  <li>The `default_value` value is used as the parameter's default if a value does not already exist.</li>
  <li>The `read_only` value specifies whether a parameter can be changed via the ROS2 API.</li>
  <li>The `description` value specifies the parameter description, which is displayed with `ros2 param describe` tool.</li>
</ol>

### validators

Parameters are run through validators during initialization and updating. If a validation fails during initialization,
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
The YAML syntax which will add the above validator to the generated code is shown below.
```yaml
validation: {
    integer_equal_value: [3]
  }
```
The `validation` maps to a YAML dictionary. Each key in the dictionary refers to a validation function generate and the value of the key specifies the functions input arguments. Validator arguments are specified with a YAML list. If a validation function is templated, then the suffix `<>` needs to be added to the function name. For example, `integer_equal_value` would be chnaged to `integer_equal_value<>`. Alternatively, the template specialization can be explicitly written, e.g `integer_equal_value<int>`.      

### Nested structure

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

The generated parameter value can then be access with `param.nest1.nest2.param_name`

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

To utilize this package, you must include it cmake project with `find_package`. Doing so will add a cmake macro `generate_parameter_library` which is used to generate the parameter library. In the example cmake file, the following should be substituted for `# additional find_package ...`
```cmake
find_package(generate_parameter_library REQUIRED)
```

After adding the executable which will depend on the generated code, `generate_parameter_library` is called function with the following arguments.

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
The above cmake additional is substituted in place of `# additional libraries ...`. Notably, the third argument of `generate_parameter_library` specifies a user provided header to include in the generated code. Validation functions defined in the header can specified in the YAML validation dictionary and run in the generated code.

## Integrate generated struct into project source code

See [example project](example/) for a complete example of how to use the generate_parameter_library.
