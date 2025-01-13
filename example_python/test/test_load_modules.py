# -*- coding: utf-8 -*-
import generate_parameter_module_example

print(f'Imported module from: {generate_parameter_module_example.__file__}')

import generate_parameter_module_example.admittance_parameters  # noqa: E402

print('Imported generated parameter module')

import generate_parameter_module_example.minimal_publisher  # noqa: E402

print('Imported minimal publisher module')
