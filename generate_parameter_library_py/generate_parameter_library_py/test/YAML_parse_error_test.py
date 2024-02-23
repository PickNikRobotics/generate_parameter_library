# -*- coding: utf-8 -*-
# Copyright 2022 PickNik, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
from unittest.mock import patch
import sys
import os

from ament_index_python.packages import get_package_share_path
from generate_parameter_library_py.generate_cpp_header import run as run_cpp
from generate_parameter_library_py.generate_python_module import run as run_python
from generate_parameter_library_py.generate_markdown import run as run_md
from generate_parameter_library_py.parse_yaml import YAMLSyntaxError
from generate_parameter_library_py.generate_cpp_header import parse_args


def set_up(yaml_test_file):
    full_file_path = os.path.join(
        get_package_share_path('generate_parameter_library_py'), 'test', yaml_test_file
    )
    testargs = [sys.argv[0], '/tmp/' + yaml_test_file + '.h', full_file_path]

    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        output_file = args.output_cpp_header_file
        yaml_file = args.input_yaml_file
        validate_header = args.validate_header
        run_cpp(output_file, yaml_file, validate_header)

    testargs = [sys.argv[0], '/tmp/' + yaml_test_file + '.py', full_file_path]

    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        output_file = args.output_cpp_header_file
        yaml_file = args.input_yaml_file
        validate_header = args.validate_header
        run_python(output_file, yaml_file, validate_header)

    testargs = [sys.argv[0], '/tmp/' + yaml_test_file + '.md', full_file_path]

    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        output_file = args.output_cpp_header_file
        yaml_file = args.input_yaml_file
        validate_header = args.validate_header
        run_md(yaml_file, output_file, 'markdown')

    testargs = [sys.argv[0], '/tmp/' + yaml_test_file + '.rst', full_file_path]

    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        output_file = args.output_cpp_header_file
        yaml_file = args.input_yaml_file
        validate_header = args.validate_header
        run_md(yaml_file, output_file, 'rst')


# class TestViewValidCodeGen(unittest.TestCase):
@pytest.mark.parametrize(
    'test_input,expected',
    [
        (file_name, YAMLSyntaxError)
        for file_name in [
            'wrong_default_type.yaml',
            'missing_type.yaml',
            'invalid_syntax.yaml',
            'invalid_parameter_type.yaml',
        ]
    ],
)
def test_expected(test_input, expected):
    with pytest.raises(expected) as e:
        yaml_test_file = test_input
        set_up(yaml_test_file)
    print(e.value)


def test_parse_valid_parameter_file():
    try:
        yaml_test_file = 'valid_parameters.yaml'
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f'failed to parse valid file, reason:{e}'


def test_parse_valid_parameter_file_including_none_type():
    try:
        yaml_test_file = 'valid_parameters_with_none_type.yaml'
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f'failed to parse valid file, reason:{e}'
