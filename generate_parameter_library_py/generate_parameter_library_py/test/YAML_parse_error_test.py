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

# import unittest
import pytest

from unittest.mock import patch
import sys
import os
from generate_parameter_library_py.main import GenerateCode, YAMLSyntaxError
from ament_index_python.packages import get_package_share_path


def set_up(yaml_test_file):
    full_file_path = os.path.join(
        get_package_share_path("generate_parameter_library_py"), "test", yaml_test_file
    )
    testargs = [sys.argv[0], "/tmp/admittance_controller.h", full_file_path]

    with patch.object(sys, "argv", testargs):
        gen_param_struct = GenerateCode()
        gen_param_struct.run()


# class TestViewValidCodeGen(unittest.TestCase):
@pytest.mark.parametrize(
    "test_input,expected",
    [
        (file_name, YAMLSyntaxError)
        for file_name in [
            "wrong_default_type.yaml",
            "missing_type.yaml",
            "invalid_syntax.yaml",
            "invalid_parameter_type.yaml",
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
        yaml_test_file = "valid_parameters.yaml"
        set_up(yaml_test_file)
    except Exception as e:
        assert False, f"failed to parse valid file, reason:{e}"
