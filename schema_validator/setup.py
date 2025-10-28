# -*- coding: utf-8 -*-
from setuptools import setup, find_packages

setup(
    name='generate-parameter-library-schema-validator',
    version='0.1.0',
    description='Schema validator for generate_parameter_library YAML files',
    author='Greenroom Robotics',
    py_modules=['validate_schema'],
    install_requires=[
        'pyyaml>=5.0',
        'jsonschema>=3.0',
    ],
    entry_points={
        'console_scripts': [
            'validate-parameter-schema=validate_schema:main',
        ],
    },
    python_requires='>=3.6',
)
