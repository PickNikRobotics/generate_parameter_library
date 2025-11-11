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


def test_control_modes_nested_structures():
    """Test nested structures within mapped parameters.

    This tests the specific issue where parameters nested within mapped structures
    (like control_modes.__map_control_mode_ids.fixed_heading.enabled) were not
    generating loop iteration code correctly.
    """

    control_modes_yaml_content = """autopilot_params:
  active_control_mode:
    type: string
    default_value: "dynamic_heading"
    description: "The current active control mode"

  control_mode_ids:
    type: string_array
    default_value: ["dynamic_heading", "fixed_heading"]
    description: "List of available control modes"

  control_modes:
    __map_control_mode_ids:
      use_trajectory:
        type: bool
        default_value: true
        description: "Use generated trajectory as control reference"

      fixed_heading:
        enabled:
          type: bool
          default_value: false
          description: "Enable fixed heading control"
        angle:
          type: double
          default_value: 0.0
          description: "Fixed heading angle in degrees"
          validation:
            bounds<>: [0.0, 360.0]
"""

    with tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', delete=False
    ) as yaml_file:
        yaml_file.write(control_modes_yaml_content)
        yaml_file.flush()

        with tempfile.NamedTemporaryFile(
            mode='w', suffix='.py', delete=False
        ) as output_file:
            try:
                run_python(output_file.name, yaml_file.name, 'test_validate.hpp')

                with open(output_file.name, 'r') as f:
                    generated_code = f.read()

                # Check for proper loop generation
                assert (
                    'for value_1 in updated_params.control_mode_ids:' in generated_code
                ), 'Should generate loop over control_mode_ids'

                # Check that we're using get_entry correctly
                assert (
                    'updated_params.control_modes.get_entry(value_1)' in generated_code
                    or 'entry = updated_params.control_modes.get_entry(value_1)' in generated_code
                ), 'Should use get_entry to access mapped parameters'

                # Get all parameter name lines
                lines = generated_code.split('\n')
                param_name_lines = [
                    line
                    for line in lines
                    if 'param_name' in line and 'control_modes' in line
                ]

                # Assert there are parameter name lines
                assert (
                    len(param_name_lines) > 0
                ), 'Should have found parameter name lines for control_modes'

                # Check for expected parameter patterns (using {value_1} for dynamic substitution)
                expected_patterns = [
                    'control_modes.{value_1}.use_trajectory',
                    'control_modes.{value_1}.fixed_heading.enabled',
                    'control_modes.{value_1}.fixed_heading.angle',
                ]

                for pattern in expected_patterns:
                    pattern_found = any(pattern in line for line in param_name_lines)
                    assert (
                        pattern_found
                    ), f"Expected pattern '{pattern}' not found in generated code.\nParam lines:\n" + '\n'.join(param_name_lines[:5])

                # Ensure no double dots in parameter names
                for line in param_name_lines:
                    assert (
                        '..' not in line
                    ), f'Found double dots in parameter name: {line.strip()}'

                # Check that we're accessing nested parameters via entry, not via __map_
                entry_access_lines = [
                    line for line in lines
                    if 'entry.' in line and ('fixed_heading' in line)
                ]
                assert (
                    len(entry_access_lines) > 0
                ), 'Should access nested parameters via entry variable'

                # Verify no direct access to __map_control_mode_ids
                wrong_access_lines = [
                    line for line in lines
                    if 'control_modes.__map_control_mode_ids' in line
                ]
                # Filter out comments
                wrong_access_lines = [
                    line for line in wrong_access_lines
                    if not line.strip().startswith('#')
                ]
                assert (
                    len(wrong_access_lines) == 0
                ), f'Should not directly access __map_control_mode_ids. Found:\n' + '\n'.join(wrong_access_lines)

            finally:
                # Clean up temporary files
                os.unlink(yaml_file.name)
                os.unlink(output_file.name)
