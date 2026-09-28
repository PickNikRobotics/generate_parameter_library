# Custom validators

## C++

Validators are functions that return a `tl::expected<void, std::string>` type and accept a `rclcpp::Parameter const&` as their first argument and any number of arguments after that can be specified in YAML.
Validators are C++ functions defined in a header file similar to the example shown below.

Here is an example custom validator.

```c++
#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <tl/expected.hpp>

namespace my_project {

tl::expected<void, std::string> integer_equal_value(
    rclcpp::Parameter const& parameter, int expected_value) {
  auto param_value = parameter.as_int();
  if (param_value != expected_value) {
    return tl::make_unexpected(fmt::format(
        "Invalid value {} for parameter {}. Expected {}",
        param_value, parameter.get_name(), expected_value));
  }
  return {};
}

}  // namespace my_project
```

Add it to `CMakeLists.txt`

```cmake
generate_parameter_library(
  turtlesim_parameters # cmake target name for the parameter library
  src/turtlesim_parameters.yaml # path to input yaml file
  src/example_validators.hpp # path to the custom validator
)
```

To configure a parameter to be validated with the custom validator function `integer_equal_value` with an `expected_value` of `3` add this to the YAML.
```yaml
validation:
  "my_project::integer_equal_value": [3]
```

## Python

Pass a Python module name as `validation_module` to `generate_parameter_module` in `setup.py`. Define functions that accept an `rclpy.parameter.Parameter` followed by the YAML arguments. Return an empty string on success or an error message on failure.

```python
def integer_equal_value(parameter, expected_value):
    if parameter.value != expected_value:
        return f"Expected {expected_value} for {parameter.name}"
    return ""
```

The generator imports your module as `custom_validators`. Reference the function using that prefix in YAML:

```yaml
validation:
  "custom_validators::integer_equal_value": [3]
```

See [Python usage](../python.md) for generation setup.
