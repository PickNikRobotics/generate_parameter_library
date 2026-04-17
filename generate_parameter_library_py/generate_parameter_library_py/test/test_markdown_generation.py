# Copyright 2026 Austrian Institute of Technology GmbH (AIT)
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

import os
import sys
from unittest.mock import patch

from ament_index_python.packages import get_package_share_path
from generate_parameter_library_py.generate_cpp_header import parse_args
from generate_parameter_library_py.generate_markdown import run as run_md


def generate_markdown(yaml_test_file):
    full_file_path = os.path.join(
        get_package_share_path('generate_parameter_library_py'), 'test', yaml_test_file
    )
    output_file = '/tmp/' + yaml_test_file + '.md'
    testargs = [sys.argv[0], output_file, full_file_path]
    with patch.object(sys, 'argv', testargs):
        args = parse_args()
        run_md(args.input_yaml_file, args.output_cpp_header_file, 'markdown')
    with open(output_file) as f:
        return f.read()


def test_none_type_parameters_are_included_but_not_typed():
    """Parameters of type 'none' should appear as headings but without a Type line."""
    markdown = generate_markdown('valid_parameters_with_none_type.yaml')

    assert '## some_external_parameter' in markdown
    assert '* Type: `none`' not in markdown


def test_element_bounds_renders_both_lower_and_upper():
    """element_bounds<> validation must show both lower and upper bound values."""
    markdown = generate_markdown('valid_parameters.yaml')

    # admittance.mass uses element_bounds<>: [ 0.0001, 1000000.0 ]
    assert 'each element of array must be within bounds [0.0001, 1000000.0]' in markdown
    # admittance.damping_ratio uses element_bounds<>: [ 0.1, 10.0 ]
    assert 'each element of array must be within bounds [0.1, 10.0]' in markdown
    # admittance.stiffness uses element_bounds (no <>): [ 0.0001, 100000.0 ]
    assert 'each element of array must be within bounds [0.0001, 100000.0]' in markdown
