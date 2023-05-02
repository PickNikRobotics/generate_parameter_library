from setuptools import setup
from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = 'generate_parameter_module_example'

# set module_name and
module_name = "admittance_parameters"
yaml_file = "generate_parameter_module_example/parameters.yaml"
generate_parameter_module(module_name, yaml_file)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paul',
    maintainer_email='paulgesel@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = generate_parameter_module_example.minimal_publisher:main'
        ],
    },
)
