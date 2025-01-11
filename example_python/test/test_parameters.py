import generate_parameter_module_example
print(f'Imported file from: {generate_parameter_module_example.__file__}')
print('OK 1')

import generate_parameter_module_example.admittance_parameters  # noqa: E402

print('OK 2')
