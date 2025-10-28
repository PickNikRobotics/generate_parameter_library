# Parameter Schema Validator

A pre-commit hook to validate `generate_parameter_library` YAML files against the JSON schema.

## Installation

### As a pre-commit hook

Add this to your `.pre-commit-config.yaml`:

```yaml
repos:
  - repo: https://github.com/Greenroom-Robotics/generate_parameter_library
    rev: main  # Use a specific tag/version in production
    hooks:
      - id: validate-parameter-schema
```

Then install the hook:

```bash
pre-commit install
```

### Standalone usage

Install dependencies:

```bash
pip install pyyaml jsonschema
```

Run the validator:

```bash
python schema_validator/validate_schema.py path/to/your/parameters.yaml
```

Or with a custom schema:

```bash
python schema_validator/validate_schema.py --schema path/to/schema.json path/to/parameters.yaml
```

## What it validates

The validator checks:

1. **YAML syntax**: Ensures the file is valid YAML
2. **Single root element**: The YAML must have exactly one root element (the namespace)
3. **Required fields**: Each parameter must have a `type` field
4. **Valid types**: Type must be one of the supported parameter types
5. **Valid field names**: Only recognized fields are allowed (type, default_value, description, read_only, validation, additional_constraints)
6. **Nested structure**: Validates nested parameter groups and mapped parameters
7. **Validation syntax**: Ensures validation rules follow the correct structure

## Example output

Successful validation:
```
✓ config/parameters.yaml is valid
```

Failed validation:
```
❌ Validation failed for config/parameters.yaml:
  Path: my_controller -> pid -> p
  Error: 'type' is a required property
  Value: {'default_value': 1.0, 'description': 'Proportional gain'}
```

## Limitations

Note that this validator checks the **structure** of the YAML file, but does not perform:

- Type checking of default values against declared types (use generate_parameter_library for this)
- Custom validator verification
- Deep semantic validation

For complete validation, always run `generate_parameter_library` during your build process.
