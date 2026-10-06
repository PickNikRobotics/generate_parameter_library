# generate_parameter_library

Generate C++ and Python code for ROS 2 parameter declaration, access, validation, and updates from declarative YAML.
The generated `Params` structure holds your values, while `ParamListener` handles interaction with the node's parameters.

## Find what you need

- **New to the library?** Follow [Getting started](getting-started.md).
- **Writing a parameter schema?** Read [YAML syntax](yaml-syntax.md).
- **Choosing a validator?** Open the [validator reference](validators/index.md), organized by scalar, string, and array types.
- **Checking every value in an array?** Use [`element_bounds<>` and other array validators](validators/arrays.md).
- **Integrating generated code?** See [C++ usage](cpp.md) or [Python usage](python.md).
- **Looking for a working package?** Explore the [examples](examples.md).

Use the search button or press **S** to search the book, including validator names such as `bounds` and `element_bounds`.

This book describes the `main` branch. For distribution-specific package documentation, see the links in the [repository README](https://github.com/PickNikRobotics/generate_parameter_library#build-status).
