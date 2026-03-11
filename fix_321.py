# example_python/generate_parameter_module_example/custom_validation.py

def validate_positive_integer(value):
    if not isinstance(value, int) or value <= 0:
        raise ValueError("Value must be a positive integer")

def validate_string_length(value, max_length):
    if not isinstance(value, str) or len(value) > max_length:
        raise ValueError(f"String length must be less than or equal to {max_length}")

def validate_custom_parameter(param_name, param_value):
    if param_name == "max_speed":
        validate_positive_integer(param_value)
    elif param_name == "robot_name":
        validate_string_length(param_value, 50)
    else:
        raise ValueError(f"Unknown parameter: {param_name}")