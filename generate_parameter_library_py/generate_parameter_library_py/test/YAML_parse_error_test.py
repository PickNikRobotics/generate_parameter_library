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
from generate_parameter_library_py.parse_yaml import YAMLSyntaxError, get_dynamic_mapped_parameter
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


@pytest.mark.parametrize(
    "yaml_test_file",
    [
        ("valid_parameters.yaml"),
        ("valid_parameters_with_none_type.yaml"),
        ("nested_map_test.yaml"),
        ("nested_map_keys.yaml"),
    ],
)
def test_parse_valid_parameter_files(yaml_test_file):
    try:
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f'failed to parse valid file, reason:{e}'

@pytest.mark.parametrize(
    "param_name,declared_params,expected",
    [
        # entries is inside nested -> resolve to nested.entries
        ("nested.__map_entries.value", {"nested.entries"}, ["nested.entries"]),
        # items is inside level1.level2 -> resolve to level1.level2.items
        ("level1.level2.__map_items.param", {"level1.level2.items"}, ["level1.level2.items"]),
        # keys is at root -> resolve to bare name
        ("__map_keys.value", {"keys"}, ["keys"]),
        # multi-level maps with keys at root -> all bare names
        (
            "__map_level1.__map_level2.__map_level3.value",
            {"level1", "level2", "level3"},
            ["level1", "level2", "level3"],
        ),
        # keys at root but __map_ inside a struct (like pid.__map_joints)
        ("pid.__map_joints.p", {"joints"}, ["joints"]),
        # nested struct with keys at root (like nested_dynamic.__map_joints)
        ("nested_dynamic.__map_joints.__map_dof_names.nested", {"joints", "dof_names"}, ["joints", "dof_names"]),
    ],
)
def test_get_dynamic_mapped_parameter_nested(param_name, declared_params, expected):
    """Test that get_dynamic_mapped_parameter returns correct paths for nested maps."""
    result = get_dynamic_mapped_parameter(param_name, declared_params)
    assert result == expected
