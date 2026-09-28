# Parameter documentation

In some cases, parameters might be unknown only at compile-time, and cannot be part of the generated C++ code. However, for documentation purpose of such parameters, the type `none` was introduced.

Parameters with `none` type won't generate any C++ code, but can exist to describe the expected name or namespace, that might be declared by an external piece of code and used in an override.

A typical use case is a controller, loading pluginlib-based filters, that themselves require (and declare) parameters in a known structure.

Example of declarative YAML

```yaml
force_torque_broadcaster_controller:
  sensor_name:
    type: string
    default_value: ""
    description: "Name of the sensor used as prefix for interfaces if there are no individual interface names defined."
  frame_id:
    type: string
    default_value: ""
    description: "Sensor's frame_id in which values are published."
  sensor_filter_chain:
    type: none
    description: "Map of parameters that defines a filter chain, containing filterN as key and underlying map of parameters needed for a specific filter. See <some docs> for more details."
```

Example of parameters for that controller

```yaml
force_torque_broadcaster_controller:
  ros__parameters:
    sensor_name: "fts_sensor"
    frame_id: "fts_sensor_frame"
    sensor_filter_chain:
      filter1:
        type: "control_filters/LowPassFilterWrench"
        name: "low_pass_filter"
        params:
          sampling_frequency: 200.0
          damping_frequency: 50.0
          damping_intensity: 1.0
```

## Generate Markdown documentation

Using generate_parameter_library you can generate a Markdown-file for your `parameters.yaml` file.
```
generate_parameter_library_markdown --input_yaml example/src/parameters.yaml --output_markdown_file parameters.md
```

This will generate a file `parameters.md` in the current folder that contains a markdown
representation of the `parameters.yaml` file that you can directly include into your documentation.
