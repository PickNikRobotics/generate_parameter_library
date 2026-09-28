# Built-in validators

- [Scalar validators](scalars.md): numeric bounds and allowed values.
- [String validators](strings.md): length and allowed values.
- [Array validators](arrays.md): element bounds, length, uniqueness, and membership.

## Scalar bounds versus array element bounds

Use `bounds<>` for a single `int` or `double`. Use `element_bounds<>` for each value in an `int_array` or `double_array`. Array length is a separate constraint, such as `fixed_size<>`.

```yaml
controller:
  gain:
    type: double
    default_value: 1.0
    validation:
      bounds<>: [0.0, 10.0]
  gains:
    type: double_array
    default_value: [1.0, 2.0, 3.0]
    validation:
      element_bounds<>: [0.0, 10.0]
      fixed_size<>: [3]
```

## Validator syntax

Validators are C++ functions that take arguments represented by a key-value pair in yaml.
The key is the name of the function.
The value is an array of values that are passed in as parameters to the function.
If the function does not take any values you write `null` or `[]` for the value.

```yaml
joint_trajectory_controller:
  command_interfaces:
    type: string_array
    description: "Names of command interfaces to claim"
    validation:
      size_gt<>: [0]
      unique<>: null
      subset_of<>: [["position", "velocity", "acceleration", "effort",]]
```

Above are validations for `command_interfaces` from `ros2_controllers`.
This will require this string_array to have these properties:

* There is at least one value in the array
* All values are unique
* Values are only in the set `["position", "velocity", "acceleration", "effort",]`

You will note that some validators have a suffix of `<>`, this tells the code generator to pass the C++ type of the parameter as a function template.
Some of these validators work only on value types, some on string types, and others on array types.
The reference pages above list the built-in functions by parameter type.


The generated C++ header includes `<rsl/parameter_validators.hpp>`, which provides the built-in validators. Older `parameter_traits` header paths are obsolete. For project-specific checks, see [custom validators](custom.md).
