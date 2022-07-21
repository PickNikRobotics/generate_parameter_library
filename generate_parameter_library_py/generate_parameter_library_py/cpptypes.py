#!/usr/bin/env python3

from typeguard import typechecked
import os
from main import GenParamStruct, Buffer
from jinja2 import Template


@typechecked
def pascal_case(string: str):
    words = string.split("_")
    return "".join(w.title() for w in words)


class DeclareParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_description: str, parameter_read_only: bool, parameter_type: str):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = parameter_type
        self.template = ""

    def __str__(self):
        return ""


class VariableDeclaration:
    @typechecked
    def __init__(self, variable_type: str, variable_name: str, value: any):
        self.variable_type = variable_type
        self.variable_name = variable_name
        self.value = value

    def __str__(self):
        type_str = self.variable_type
        value_str = self.value
        declare_str = f"{type_str} {self.variable_name} = {value_str};"
        return declare_str


class Struct:
    @typechecked
    def __init__(self, struct_name: str, fields: list[VariableDeclaration]):
        self.struct_name = struct_name
        self.fields = fields
        self.sub_structs = []
        self.template = ""

    @typechecked
    def add_field(self, field: VariableDeclaration):
        self.fields.append(field)

    @typechecked
    def add_sub_struct(self, sub_struct):
        self.sub_structs.append(sub_struct)

    def inner_content(self):
        content = Buffer()
        for field in self.fields:
            content += str(field) + '\n'
        for sub_struct in self.sub_structs:
            content += str(sub_struct) + '\n'

        return str(content)

    def __str__(self):
        sub_struct_str = Buffer()
        for sub_struct in self.sub_structs:
            sub_struct_str += str(sub_struct) + '\n'

        field_str = Buffer()
        for field in self.fields:
            field_str += str(field) + '\n'

        data = {'struct_name': pascal_case(self.struct_name),
                'struct_instance': self.struct_name.lower(),
                'struct_fields': str(field_str),
                'sub_structs': str(sub_struct_str)}

        j2_template = Template(GenParamStruct.templates['declare_struct'])
        code = j2_template.render(data, trim_blocks=True)
        return code


class ValidationFunction:
    @typechecked
    def __init__(self, function_name: str, arguments: list[any]):
        self.function_name = function_name
        self.arguments = arguments

    def __str__(self):
        return ""


class ParameterValidation:
    @typechecked
    def __init__(self, invalid_effect: str, valid_effect: str):
        self.invalid_effect = invalid_effect
        self.valid_effect = valid_effect
        self.validation_functions = []

    def add_validation_function(self, validation_function: ValidationFunction):
        self.validation_functions.append(validation_function)

    def __str__(self):
        return ""


class UpdateParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []

    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        return ""


class DeclareParameterSet:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []

    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        return ""
