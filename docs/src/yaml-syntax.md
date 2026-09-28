# YAML syntax

## Cpp namespace
The root element of the YAML file determines the namespace used in the generated C++ code.
We use this to put the `Params` struct in the same namespace as your C++ code.

```yaml
cpp_namespace:
# additionally fields  ...
```

## Parameter definition
The YAML syntax can be thought of as a tree since it allows for arbitrary nesting of key-value pairs.
For clarity, the last non-nested value is referred to as a leaf.
A leaf represents a single parameter and has the following format.

```yaml
cpp_namespace:
  param_name:
    type: int
    default_value: 3
    read_only: true
    additional_constraints: "{ type: 'number', multipleOf: 3 }"
    description: "A read-only  integer parameter with a default value of 3"
    validation:
      # validation functions ...
```

A parameter is a YAML dictionary with the only required key being `type`.

| Field                  | Description                                                                                                    |
| ---------------------- | -------------------------------------------------------------------------------------------------------------- |
| type                   | The type (string, double, etc) of the parameter.                                                               |
| default_value          | Value for the parameter if the user does not specify a value.                                                  |
| read_only              | Can only be set at launch and are not dynamic.                                                                 |
| description            | Displayed by `ros2 param describe`.                                                                            |
| validation             | Dictionary of validation functions and their parameters.                                                       |
| additional_constraints | Additional constraints that end up on the ParameterDescriptor but are not used for validation by this package. |

The types of parameters in ros2 map to C++ types.

| Parameter Type  | C++ Type                   |
| --------------- | -------------------------- |
| string          | `std::string`              |
| double          | `double`                   |
| int             | `int64_t`                  |
| bool            | `bool`                     |
| string_array    | `std::vector<std::string>` |
| double_array    | `std::vector<double>`      |
| int_array       | `std::vector<int64_t>`         |
| bool_array      | `std::vector<bool>`        |
| string_fixed_XX | `rsl::StaticString<XX>`      |
| none            | NO CODE GENERATED          |

Fixed-size types are denoted with a suffix `_fixed_XX`, where `XX` is the desired size.
The corresponding C++ type is a data wrapper class for conveniently accessing the data.
Note that any fixed size type will automatically use a `size_lt` validator. See the [validator reference](validators/index.md).

The purpose of the `none` type is purely documentation, and won't generate any C++ code. See [Parameter documentation](parameter-documentation.md) for details.

## Nested structures
After the top-level key, every subsequent non-leaf key will generate a nested C++ struct. The struct instance will have
the same name as the key.

```yaml
cpp_name_space:
  nest1:
    nest2:
      param_name: # this is a leaf
        type: string_array
```

The generated parameter value can then be accessed with `params.nest1.nest2.param_name`
