# FAQ

Q. What happens if I declare a parameter twice? Will I get an error at runtime?

A. The declare routine that is generated checks to see if each parameter has been declared first before declaring it. Because of this you can declare a parameter twice but it will only have the properties of the first time you declared it. Here is some example generated code.
```cpp
if (!parameters_interface_->has_parameter(prefix_ + "scientific_notation_num")) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Test scientific notation";
    descriptor.read_only = false;
    auto parameter = to_parameter_value(updated_params.scientific_notation_num);
    parameters_interface_->declare_parameter(prefix_ + "scientific_notation_num", parameter, descriptor);
}
```

Q: How do I log when parameters change?

A. The generated library outputs debug logs whenever a parameter is read from ROS.
