#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pre-commit hook to validate generate_parameter_library YAML files against JSON schema.
"""

import argparse
import json
import sys
from pathlib import Path
from typing import List, Optional

try:
    import yaml
    from jsonschema import Draft7Validator
    from jsonschema.exceptions import SchemaError
except ImportError as e:
    print(f"Error: Required package not installed: {e}", file=sys.stderr)
    print('Install with: pip install pyyaml jsonschema', file=sys.stderr)
    sys.exit(1)


def load_schema(schema_path: Path) -> dict:
    """Load the JSON schema from file."""
    try:
        with open(schema_path, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        print(f"Error: Schema file not found: {schema_path}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Error: Invalid JSON schema: {e}", file=sys.stderr)
        sys.exit(1)


def load_yaml(yaml_path: Path) -> Optional[dict]:
    """Load YAML file."""
    try:
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)
    except yaml.YAMLError as e:
        print(f"Error parsing {yaml_path}: {e}", file=sys.stderr)
        return None
    except FileNotFoundError:
        print(f"Error: File not found: {yaml_path}", file=sys.stderr)
        return None


def validate_yaml_against_schema(
    yaml_data: dict, schema: dict, yaml_path: Path
) -> bool:
    """Validate YAML data against JSON schema."""
    try:
        validator = Draft7Validator(schema)

        # Check if there are validation errors
        errors = list(validator.iter_errors(yaml_data))

        if errors:
            print(f"\n❌ Validation failed for {yaml_path}:", file=sys.stderr)
            for error in errors:
                # Build path to the error
                path = ' -> '.join(str(p) for p in error.path) if error.path else 'root'
                print(f"  Path: {path}", file=sys.stderr)
                print(f"  Error: {error.message}", file=sys.stderr)

                # Show the failing instance if it's not too large
                if error.instance and len(str(error.instance)) < 200:
                    print(f"  Value: {error.instance}", file=sys.stderr)
                print('', file=sys.stderr)
            return False

        return True

    except SchemaError as e:
        print(f"Error: Invalid schema: {e}", file=sys.stderr)
        sys.exit(1)


def validate_additional_constraints(
    node: dict, path: str, yaml_path: Path, ui_schema: Optional[dict]
) -> bool:
    """
    Recursively validate additional_constraints JSON content against UI schema.
    Returns True if valid or if ui_schema is None.
    """
    if ui_schema is None:
        return True

    all_valid = True

    # Check if this node has additional_constraints
    if 'additional_constraints' in node:
        constraints_str = node['additional_constraints']
        try:
            # Parse the JSON string
            constraints_data = json.loads(constraints_str)

            # Validate against UI schema
            from jsonschema import Draft7Validator

            validator = Draft7Validator(ui_schema)
            errors = list(validator.iter_errors(constraints_data))

            if errors:
                print(
                    f"\n❌ additional_constraints validation failed at {path} in {yaml_path}:",
                    file=sys.stderr,
                )
                for error in errors:
                    error_path = (
                        ' -> '.join(str(p) for p in error.path)
                        if error.path
                        else 'root'
                    )
                    print(f"  Path: {error_path}", file=sys.stderr)
                    print(f"  Error: {error.message}", file=sys.stderr)
                print(f"  Constraints value: {constraints_str}", file=sys.stderr)
                print('', file=sys.stderr)
                all_valid = False

        except json.JSONDecodeError as e:
            print(
                f"\n❌ additional_constraints is not valid JSON at {path} in {yaml_path}:",
                file=sys.stderr,
            )
            print(f"  Error: {e}", file=sys.stderr)
            print(f"  Value: {constraints_str}", file=sys.stderr)
            print('', file=sys.stderr)
            all_valid = False

    # Recursively check nested parameters
    for key, value in node.items():
        if isinstance(value, dict) and key not in [
            'type',
            'default_value',
            'description',
            'read_only',
            'validation',
            'additional_constraints',
        ]:
            nested_path = f"{path} -> {key}" if path else key
            if not validate_additional_constraints(
                value, nested_path, yaml_path, ui_schema
            ):
                all_valid = False

    return all_valid


def validate_parameter_structure(yaml_data: dict, yaml_path: Path) -> bool:
    """
    Additional validation beyond JSON schema:
    - Ensure only one root element
    - Check type/default_value consistency
    """
    if not isinstance(yaml_data, dict):
        print(f"Error in {yaml_path}: Root must be a dictionary", file=sys.stderr)
        return False

    # Check for single root element
    if len(yaml_data) != 1:
        print(
            f"Error in {yaml_path}: YAML must have exactly one root element (namespace), "
            f"found {len(yaml_data)}: {list(yaml_data.keys())}",
            file=sys.stderr,
        )
        return False

    return True


def main(argv: Optional[List[str]] = None) -> int:
    """Main entry point for the validator."""
    parser = argparse.ArgumentParser(
        description='Validate generate_parameter_library YAML files against JSON schema'
    )
    parser.add_argument('files', nargs='+', type=Path, help='YAML files to validate')
    parser.add_argument(
        '--schema',
        type=Path,
        default=Path(__file__).parent / 'parameter_schema.json',
        help='Path to JSON schema file (default: parameter_schema.json)',
    )
    parser.add_argument(
        '--ui-schema',
        action='store_true',
        help='Validate additional_constraints JSON content against alpha_ui_schema.json',
    )

    args = parser.parse_args(argv)

    # Load schema once
    schema = load_schema(args.schema)

    # Load UI schema if flag is set
    ui_schema = None
    if args.ui_schema:
        ui_schema_path = Path(__file__).parent / 'alpha_ui_schema.json'
        ui_schema = load_schema(ui_schema_path)

    all_valid = True

    for yaml_file in args.files:
        # Load YAML
        yaml_data = load_yaml(yaml_file)
        if yaml_data is None:
            all_valid = False
            continue

        # Validate structure (single root, etc.)
        if not validate_parameter_structure(yaml_data, yaml_file):
            all_valid = False
            continue

        # Validate against schema
        if not validate_yaml_against_schema(yaml_data, schema, yaml_file):
            all_valid = False
            continue

        # Validate additional_constraints if UI schema provided
        if ui_schema:
            # Get the root namespace
            root_namespace = list(yaml_data.keys())[0]
            if not validate_additional_constraints(
                yaml_data[root_namespace], root_namespace, yaml_file, ui_schema
            ):
                all_valid = False
                continue

        print(f"✓ {yaml_file} is valid")

    return 0 if all_valid else 1


if __name__ == '__main__':
    sys.exit(main())
