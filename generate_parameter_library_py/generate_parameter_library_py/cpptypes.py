#!/usr/bin/env python3

from typeguard import typechecked
import os
from main import *
from jinja2 import Template


@typechecked
def pascal_case(string: str):
    words = string.split("_")
    return "".join(w.title() for w in words)


class DeclareParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_description: str, parameter_read_only: bool, parameter_type: str, default_value: any):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = parameter_type
        self.default_value = default_value

    def __str__(self):
        parameter_type = "PARAMETER_" + self.parameter_type.upper()
        if self.default_value is None:
            default_value = ""
        else:
            default_value = "not_empty"

        data = {'parameter_name': self.parameter_name,
                'parameter_type': parameter_type,
                'parameter_description': self.parameter_description,
                'parameter_read_only': bool_to_str(self.parameter_read_only),
                'default_value': default_value
                }

        j2_template = Template(GenParamStruct.templates['declare_parameter'])
        code = j2_template.render(data, trim_blocks=True)
        return code



class VariableDeclaration:
    @typechecked
    def __init__(self, variable_type: str, variable_name: str, value: any):
        self.variable_type = variable_type
        self.variable_name = variable_name
        self.value = value

    def __str__(self):
        val_func = get_val_to_cpp_str_func(self.variable_type)
        type_str = get_cpp_type(self.variable_type)
        if self.value is None:
            declare_str = f"{type_str} {self.variable_name};"
        elif isinstance(self.value, list):
            value_str = '{'
            for val in self.value[:-1]:
                value_str += val_func(val) + ", "
            if len(self.value) > 0:
                value_str += val_func(self.value[-1])
            value_str += '}'

            declare_str = f"{type_str} {self.variable_name} = {value_str};"
        else:
            value_str = val_func(self.value)
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

    def get_translation(self, arg):
        if isinstance(arg, int):
            val_func = int_to_str
        elif isinstance(arg, float):
            val_func = float_to_str
        elif isinstance(arg, bool):
            val_func = bool_to_str
        elif isinstance(arg, str):
            val_func = str_to_str
        return val_func

    def __str__(self):
        code = self.function_name + '(param'
        for arg in self.arguments[:-1]:
            val_func = self.get_translation(arg)
            code += ", " + val_func(arg)
        if len(self.arguments) > 0:
            val_func = self.get_translation(self.arguments[-1])
            code += ", " + val_func(self.arguments[-1])
        code += ')'

        return code


class ParameterValidation:
    @typechecked
    def __init__(self, invalid_effect: str, valid_effect: str, validation_function: ValidationFunction):
        self.invalid_effect = invalid_effect
        self.valid_effect = valid_effect
        self.validation_function = validation_function

    def __str__(self):
        data = {'validation_function': str(self.validation_function),
                'valid_effect': self.valid_effect,
                'invalid_effect': self.invalid_effect}

        j2_template = Template(GenParamStruct.templates['parameter_validation'])
        code = j2_template.render(data, trim_blocks=True)
        return code


class UpdateParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []

    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        parameter_validations_str = Buffer()
        for parameter_validation in self.parameter_validations:
            parameter_validations_str += str(parameter_validation) + '\n'

        data = {'parameter_name': self.parameter_name,
                'parameter_validations': str(parameter_validations_str),
                'parameter_as_function': self.parameter_as_function}

        j2_template = Template(GenParamStruct.templates['update_parameter'])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DeclareParameterSet:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []

    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        parameter_validations_str = Buffer()
        for parameter_validation in self.parameter_validations:
            parameter_validations_str += str(parameter_validation) + '\n'

        data = {'parameter_name': self.parameter_name,
                'parameter_validations': str(parameter_validations_str),
                'parameter_as_function': self.parameter_as_function}

        j2_template = Template(GenParamStruct.templates['declare_parameter_set'])
        code = j2_template.render(data, trim_blocks=True)
        return code
