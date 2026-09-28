# Mapped parameters

You can use parameter maps, where a map with keys from another `string_array` parameter is created. Add the `__map_` prefix followed by the key parameter name as follows:

```yaml
cpp_name_space:
  joints:
    type: string_array
    default_value: ["joint1", "joint2", "joint3"]
    description: "specifies which joints will be used by the controller"
  interfaces:
    type: string_array
    default_value: ["position", "velocity", "acceleration"]
    description: "interfaces to be used by the controller"
  # nested mapped example
  gain:
    __map_joints: # create a map with joints as keys
      __map_interfaces:  # create a map with interfaces as keys
        value:
          type: double
  # simple mapped example
  pid:
    __map_joints: # create a map with joints as keys
      values:
        type: double_array
```

The generated parameter value for the nested map example can then be accessed with:

**C++**

```c++
params.gain.joints_map.at("joint1").interfaces_map.at("position").value
```

**Python**

```python
params.gain.get_entry("joint1").get_entry("position").value
```

## Key array scope resolution

The `key` used by a `__map_<key>` segment does not need to be defined at the root namespace level. It can also be a **sibling** within the same struct, or defined anywhere in a parent scope.
This allows you to co-locate the key array alongside the map it controls:

```yaml
cpp_name_space:
  # key array defined as a sibling of the map that uses it
  nested_map:
    entries:
      type: string_array
      default_value: ["entry1", "entry2"]
      description: "Keys for the nested map"
    __map_entries: # resolved to nested_map.entries (sibling scope)
      value:
        type: double
        default_value: 1.0
        description: "A value keyed by entries"
```

> **Note:** Scope resolution searches the current struct first, then walks up to parent scopes. If the key array is not found in any scope, the bare name is used as a fallback.
