# -*- coding: utf-8 -*-
import pytest
import tempfile
import os

from generate_parameter_library_py.generate_python_module import run as run_python


def test_nested_map_parameter_names():
    """Test that nested map parameters don't generate double dots in parameter names."""

    nested_map_yaml_content = """test_namespace:
  config_groups:
    __map_group_ids:
        max_value:
          type: double
          default_value: 100.0
          validation:
            bounds<>: [-100.0, 100.0]
        __map_item_ids:
          enabled:
            type: bool
            default_value: true
          min_param:
            type: double
            default_value: -1.0
            validation:
              bounds<>: [-1.0, 1.0]
          max_param:
            type: double
            default_value: 1.0
            validation:
              bounds<>: [-1.0, 1.0]
          control_id:
            type: string
            default_value: ""
"""

    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False
    ) as yaml_file:
        yaml_file.write(nested_map_yaml_content)
        yaml_file.flush()

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.py', delete=False
        ) as output_file:
            try:
                run_python(output_file.name, yaml_file.name, 'test_validate.hpp')

                with open(output_file.name, 'r') as f:
                    generated_code = f.read()

                # Check that no parameter names have double dots
                lines = generated_code.split('\n')
                param_name_lines = [
                    line
                    for line in lines
                    if 'param_name' in line and 'config_groups' in line
                ]

                # Assert there are parameter name lines (test that our test is working)
                assert (
                    len(param_name_lines) > 0
                ), 'Should have found parameter name lines'

                # Check each param_name line for double dots
                for line in param_name_lines:
                    assert (
                        '..' not in line
                    ), f'Found double dots in parameter name: {line.strip()}'

                expected_patterns = [
                    'config_groups.{value_1}.max_value',
                    'config_groups.{value_1}.{value_2}.enabled',
                    'config_groups.{value_1}.{value_2}.min_param',
                    'config_groups.{value_1}.{value_2}.max_param',
                    'config_groups.{value_1}.{value_2}.control_id',
                ]

                for pattern in expected_patterns:
                    pattern_found = any(pattern in line for line in param_name_lines)
                    assert (
                        pattern_found
                    ), f"Expected pattern '{pattern}' not found in generated code"

            finally:
                # Clean up temporary files
                os.unlink(yaml_file.name)
                os.unlink(output_file.name)


def test_single_map_parameter_names():
    """Test that single map parameters still work correctly after the fix."""

    single_map_yaml_content = """test_namespace:
  params:
    __map_keys:
      value_a:
        type: double
        default_value: 1.0
        validation:
          gt_eq<>: [ 0 ]
      value_b:
        type: double
      value_c:
        type: double
        default_value: 1.0
"""

    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False
    ) as yaml_file:
        yaml_file.write(single_map_yaml_content)
        yaml_file.flush()

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.py', delete=False
        ) as output_file:
            try:
                run_python(output_file.name, yaml_file.name, 'test_validate.hpp')

                with open(output_file.name, 'r') as f:
                    generated_code = f.read()

                # Check that no parameter names have double dots
                lines = generated_code.split('\n')
                param_name_lines = [
                    line for line in lines if 'param_name' in line and 'params' in line
                ]

                # Assert there are parameter name lines (test that our test is working)
                assert (
                    len(param_name_lines) > 0
                ), 'Should have found parameter name lines'

                # Check each param_name line for double dots
                for line in param_name_lines:
                    assert (
                        '..' not in line
                    ), f'Found double dots in parameter name: {line.strip()}'

                expected_patterns = [
                    'params.{value_1}.value_a',
                    'params.{value_1}.value_b',
                    'params.{value_1}.value_c',
                ]

                for pattern in expected_patterns:
                    pattern_found = any(pattern in line for line in param_name_lines)
                    assert (
                        pattern_found
                    ), f"Expected pattern '{pattern}' not found in generated code"

            finally:
                # Clean up temporary files
                os.unlink(yaml_file.name)
                os.unlink(output_file.name)
