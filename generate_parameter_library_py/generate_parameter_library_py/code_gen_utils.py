#!/usr/bin/env python3

import os
from typing import Callable, Optional
from typeguard import typechecked
from jinja2 import Template
from main import GenParamStruct


# helper  classes

# Buffer help minimize string copies
class Buffer:
    def __init__(self):
        self.data_ = bytearray()

    def __iadd__(self, element):
        self.data_.extend(element.encode())
        return self

    def __str__(self):
        return self.data_.decode()


# YAMLSyntaxError standardizes compiler error messages
class YAMLSyntaxError(Exception):
    """Raised when the input value is too large"""

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


# helper functions

@typechecked
def compile_error(msg: str):
    return YAMLSyntaxError("\nERROR: " + msg)


@typechecked
def array_type(defined_type: str):
    return defined_type.__contains__("array")


@typechecked
def validate_type(defined_type: str, value):
    type_translation = {"string": str,
                        "double": float,
                        "int": int,
                        "bool": bool,
                        "string_array": str,
                        "double_array": float,
                        "int_array": int,
                        "bool_array": bool}

    if isinstance(value, list):
        if not array_type(defined_type):
            return False
        for val in value:
            if type_translation[defined_type] != type(val):
                return False
    else:
        if array_type(defined_type):
            return False
        if type_translation[defined_type] != type(value):
            return False

    return True


# value to c++ string conversion functions
@typechecked
def bool_to_str(cond: Optional[bool]):
    if cond is None:
        return None
    return "true" if cond else "false"


@typechecked
def float_to_str(num: Optional[float]):
    if num is None:
        return None
    str_num = str(num)
    if str_num == "nan":
        str_num = "std::numeric_limits<double>::quiet_NaN()"
    elif str_num == "inf":
        str_num = "std::numeric_limits<double>::infinity()"
    elif str_num == "-inf":
        str_num = "-std::numeric_limits<double>::infinity()"
    else:
        if len(str_num.split('.')) == 1:
            str_num += ".0"

    return str_num


@typechecked
def int_to_str(num: Optional[int]):
    if num is None:
        return None
    return str(num)


@typechecked
def str_to_str(s: Optional[str]):
    if s is None:
        return None
    return "\"%s\"" % s


# cpp_type, val_to_cpp_str, parameter_conversion
@typechecked
def cpp_type_from_defined_type(yaml_type: str) -> str:
    if yaml_type == 'string_array':
        cpp_type = 'std::vector<std::string>'
    elif yaml_type == 'double_array':
        cpp_type = 'std::vector<double>'
    elif yaml_type == 'int_array':
        cpp_type = 'std::vector<int>'
    elif yaml_type == 'bool_array':
        cpp_type = 'std::vector<bool>'
    elif yaml_type == 'string':
        cpp_type = 'std::string'
    elif yaml_type == 'double':
        cpp_type = 'double'
    elif yaml_type == 'integer':
        cpp_type = 'int'
    elif yaml_type == 'bool':
        cpp_type = 'bool'
    else:
        raise compile_error('invalid yaml type: %s' % type(yaml_type))

    return cpp_type


# cpp_type, val_to_cpp_str, parameter_conversion
@typechecked
def cpp_str_func_from_defined_type(yaml_type: str) -> Callable:
    if yaml_type == 'string_array':
        val_to_cpp_str = str_to_str
    elif yaml_type == 'double_array':
        val_to_cpp_str = float_to_str
    elif yaml_type == 'int_array':
        val_to_cpp_str = int_to_str
    elif yaml_type == 'bool_array':
        val_to_cpp_str = bool_to_str
    elif yaml_type == 'string':
        val_to_cpp_str = str_to_str
    elif yaml_type == 'double':
        val_to_cpp_str = float_to_str
    elif yaml_type == 'integer':
        val_to_cpp_str = int_to_str
    elif yaml_type == 'bool':
        val_to_cpp_str = bool_to_str
    else:
        raise compile_error('invalid yaml type: %s' % type(yaml_type))

    return val_to_cpp_str


@typechecked
def cpp_str_func_from_python_val(arg: any) -> Callable:
    if isinstance(arg, int):
        val_func = int_to_str
    elif isinstance(arg, float):
        val_func = float_to_str
    elif isinstance(arg, bool):
        val_func = bool_to_str
    elif isinstance(arg, str):
        val_func = str_to_str
    else:
        raise compile_error('invalid python arg type: %s' % type(arg))
    return val_func


@typechecked
def get_parameter_as_function_str(yaml_type: str) -> str:
    if yaml_type == 'string_array':
        parameter_conversion = 'as_string_array()'
    elif yaml_type == 'double_array':
        parameter_conversion = 'as_double_array()'
    elif yaml_type == 'int_array':
        parameter_conversion = 'as_integer_array()'
    elif yaml_type == 'bool_array':
        parameter_conversion = 'as_bool_array()'
    elif yaml_type == 'string':
        parameter_conversion = 'as_string()'
    elif yaml_type == 'double':
        parameter_conversion = 'as_double()'
    elif yaml_type == 'integer':
        parameter_conversion = 'as_int()'
    elif yaml_type == 'bool':
        parameter_conversion = 'as_bool()'
    else:
        raise compile_error('invalid yaml type: %s' % type(yaml_type))

    return parameter_conversion


@typechecked
def pascal_case(string: str):
    words = string.split("_")
    return "".join(w.title() for w in words)


@typechecked
def initialization_fail_validation(param_name: str) -> str:
    return f'throw rclcpp::exceptions::InvalidParameterValueException("Invalid value set during initialization for parameter {param_name}: + validation_result.error_msg());");'

@typechecked
def initialization_pass_validation(param_name: str, parameter_conversion: str) -> str:
    return f"params_.{param_name} = param.{parameter_conversion};"

@typechecked
def update_parameter_fail_validation() -> str:
    return "result.successful = false;\nbreak;"

@typechecked
def update_parameter_pass_validation() -> str:
    return ""


# Each template has a corresponding class with the str filling in the template with jinja

class DeclareParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_description: str, parameter_read_only: bool, parameter_type: str,
                 default_value: any):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = parameter_type
        self.default_value = default_value

    def __str__(self):
        parameter_type = self.parameter_type.upper()
        if self.default_value is None:
            default_value = ""
        else:
            default_value = "not_empty"

        data = {'parameter_name': self.parameter_name,
                'parameter_type': parameter_type,
                'parameter_description': self.parameter_description,
                'parameter_read_only': bool_to_str(self.parameter_read_only),
                'default_value': default_value}

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
        val_func = cpp_str_func_from_defined_type(self.variable_type)
        type_str = cpp_type_from_defined_type(self.variable_type)
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

    def __str__(self):
        code = self.function_name + '(param'
        for arg in self.arguments[:-1]:
            val_func = cpp_str_func_from_python_val(arg)
            code += ", " + val_func(arg)
        if len(self.arguments) > 0:
            val_func = cpp_str_func_from_python_val(self.arguments[-1])
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

    @typechecked
    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        parameter_validations_str = Buffer()
        for parameter_validation in self.parameter_validations:
            parameter_validations_str += str(parameter_validation)  # + '\n'

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

    @typechecked
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
