#!/usr/bin/env python3

from typeguard import typechecked


class VariableDeclaration:
    @typechecked
    def __init__(self, variable_type: str, variable_name: str, value: list[any]):
        self.variable_type = variable_type
        self.variable_name = variable_name
        self.value = value


class Struct:
    @typechecked
    def __init__(self, struct_name: str, fields: list[VariableDeclaration]):
        self.struct_name = struct_name
        self.fields = fields
        self.sub_structs = []

    @typechecked
    def add_field(self, field: VariableDeclaration):
        self.fields.append(field)

    @typechecked
    def add_sub_struct(self, sub_struct):
        self.sub_structs.append(sub_struct)


# if (param.get_name() == "{{parameter_name}}") {
# result.successful = false;
# {{declare_param_validations}}
# params_.{{parameter_name}} = param.{{parameter_as_function}};
# result.successful = true;
# }

class ValidationFunction:
    @typechecked
    def __init__(self, function_name: str, arguments: list[any]):
        self.function_name = function_name
        self.arguments = arguments


class DeclareParameterValidation:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.validation_functions = []

    def add_validation_function(self, validation_function: ValidationFunction):
        self.validation_functions.append(validation_function)


class UpdateParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.declare_param_validations = []


class DeclareParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_description: str, parameter_read_only: bool, parameter_type: str):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = parameter_type

# param = parameters_interface->get_parameter("{{parameter_name}}");
# {{declare_param_validations}}
# params_.{{parameter_name}}_ = param.{{parameter_as_function}};


class DeclareParameterSet:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str, parameter_read_only: bool, parameter_type: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function

