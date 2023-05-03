from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'generate_parameter_module_example'

# set module_name and yaml file
module_name = "admittance_parameters"
yaml_file = "generate_parameter_module_example/parameters.yaml"
generate_parameter_module(module_name, yaml_file)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Gesel',
    maintainer_email='paulgesel@gmail.com',
    description='Example usage of generate_parameter_library for a python module',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = generate_parameter_module_example.minimal_publisher:main'
        ],
    },
)
