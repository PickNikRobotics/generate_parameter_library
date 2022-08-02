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
from generate_parameter_library_py.main import GenerateCode, YAMLSyntaxError



def set_up(yaml_test_file):
    testargs = [
        "/home/paul/Downloads/colcon_ws/src/generate_parameter_library/generate_parameter_library_py/generate_parameter_library_py/main.py",
        "/tmp/admittance_controller.h",
        "/home/paul/Downloads/colcon_ws/src/generate_parameter_library/generate_parameter_library_py/generate_parameter_library_py/test/" + yaml_test_file]
    with patch.object(sys, 'argv', testargs):
        gen_param_struct = GenerateCode()
        gen_param_struct.run()
        # print(gen_param_struct)


# class TestViewValidCodeGen(unittest.TestCase):
@pytest.mark.parametrize("test_input,expected", [(file_name, YAMLSyntaxError) for file_name in ["bad.yaml"]])
def test_expected(test_input, expected):
    with pytest.raises(expected):
        yaml_test_file = test_input
        set_up(yaml_test_file)


def test_parse_valid_parameter_file():
    try:
        yaml_test_file = "parameters.yaml"
        set_up(yaml_test_file)
    except Exception:
        assert (False, "failed to parse valid file")
