# Python usage

Use the same [YAML schema](yaml-syntax.md) and [built-in validators](validators/index.md) as the C++ generator.

## Generate with setuptools

In an `ament_python` package, declare a dependency on `generate_parameter_library_py` in `package.xml` and call its helper from `setup.py`:

```python
import sys
from generate_parameter_library_py.setup_helper import generate_parameter_module

if len(sys.argv) >= 2 and sys.argv[1] != 'clean':
    generate_parameter_module(
        'turtlesim_parameters',
        'turtlesim/parameters.yaml',
        # Optional module containing custom validator functions:
        # validation_module='turtlesim.custom_validation',
    )
```

Keep your normal `setup(...)` call and package metadata. Build the package with `colcon build` and source the workspace's `install/setup.bash`.
For a Python package named `turtlesim`, use the generated module as follows:

```python
import rclpy
from rclpy.node import Node
from turtlesim import turtlesim_parameters

rclpy.init()
node = Node('turtlesim')
listener = turtlesim_parameters.ParamListener(node)
params = listener.get_params()
node.get_logger().info(f'Red background value: {params.background.r}')
```

The `background.r` field comes from the schema in [Getting started](getting-started.md).
Keep the listener alive while the node is running. To refresh a cached snapshot:

```python
if listener.is_old(params):
    params = listener.get_params()
```

## Generate with CMake

An `ament_cmake_python` package can generate a module before installing its Python package:

```cmake
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(generate_parameter_library REQUIRED)

generate_parameter_module(turtlesim_parameters
  turtlesim/parameters.yaml
  # Optional third argument: turtlesim.custom_validation
)
ament_python_install_package(${PROJECT_NAME})
ament_package()
```

See the [setuptools and CMake examples](examples.md) for complete package layouts and the [custom validator guide](validators/custom.md#python) for Python validation functions.
