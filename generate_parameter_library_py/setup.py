# -*- coding: utf-8 -*-
from setuptools import find_packages
from setuptools import setup

package_name = "generate_parameter_library_py"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ],
    install_requires=["setuptools"],
    package_data={
        "": [
            "cpp_templates/validators.hpp",
            "jinja_templates/declare_parameter",
            "jinja_templates/declare_parameter_set",
            "jinja_templates/declare_struct",
            "jinja_templates/parameter_listener",
            "jinja_templates/parameter_validation",
            "jinja_templates/update_parameter",
        ]
    },
    zip_safe=False,
    author="Paul Gesel",
    author_email="paul.gesel@picknik.ai",
    url="https://github.com/pac48/gen_param_struct",
    download_url="https://github.com/pac48/gen_param_struct/releases",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: BSD-3-Clause",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Generate the ROS parameter struct in C++ with callbacks for updating.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_parameter_library_py = generate_parameter_library_py.main:main",
        ],
    },
)
