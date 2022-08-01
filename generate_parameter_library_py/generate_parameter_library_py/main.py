#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 PickNik Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import yaml
from yaml.parser import ParserError
import sys
import os
from typing import Optional
from typeguard import typechecked
from jinja2 import Template


# YAMLSyntaxError standardizes compiler error messages
class YAMLSyntaxError(Exception):
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
def is_mapped_parameter(param_name: str):
    return param_name.__contains__("__map_")


@typechecked
def fixed_type_size(yaml_type: str):
    tmp = yaml_type.split("_")
    if len(tmp) < 3:
        return None
    if tmp[-2] != "fixed" or not tmp[-1].isdigit():
        return None
    return int(tmp[-1])


@typechecked
def is_fixed_type(yaml_type: str):
    return fixed_type_size(yaml_type) is not None


@typechecked
def get_fixed_type(yaml_type: str):
    tmp = yaml_type.split("_")
    return "".join(tmp[:-2])


@typechecked
def validate_type(defined_type: str, value):
    type_translation = {
        "string": str,
        "double": float,
        "int": int,
        "bool": bool,
        "string_array": str,
        "double_array": float,
        "int_array": int,
        "bool_array": bool,
    }

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
        if len(str_num.split(".")) == 1:
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
    return '"%s"' % s


@typechecked
def fixed_str_to_str(s: Optional[str]):
    return "{%s}" % str_to_str(s)


# cpp_type, val_to_cpp_str, parameter_conversion
@typechecked
def cpp_type_from_defined_type(yaml_type: str) -> str:
    if yaml_type == "string_array":
        cpp_type = "std::vector<std::string>"
    elif yaml_type == "double_array":
        cpp_type = "std::vector<double>"
    elif yaml_type == "int_array":
        cpp_type = "std::vector<int>"
    elif yaml_type == "bool_array":
        cpp_type = "std::vector<bool>"
    elif yaml_type == "string":
        cpp_type = "std::string"
    elif yaml_type.__contains__("string_fixed_"):
        cpp_type = f"parameter_traits::FixedSizeString<{fixed_type_size(yaml_type)}>"
    elif yaml_type == "double":
        cpp_type = "double"
    elif yaml_type == "int":
        cpp_type = "int"
    elif yaml_type == "bool":
        cpp_type = "bool"
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

    return cpp_type


@typechecked
def cpp_primitive_type_from_defined_type(yaml_type: str) -> str:
    if yaml_type == "string_array" or yaml_type.__contains__("string_fixed_"):
        cpp_type = "std::string"
    elif yaml_type == "double_array":
        cpp_type = "double"
    elif yaml_type == "int_array":
        cpp_type = "int"
    elif yaml_type == "bool_array":
        cpp_type = "bool"
    elif yaml_type == "string":
        cpp_type = "std::string"
    elif yaml_type == "double":
        cpp_type = "double"
    elif yaml_type == "int":
        cpp_type = "int"
    elif yaml_type == "bool":
        cpp_type = "bool"
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

    return cpp_type


# TODO type checking fails here
# @typechecked
def cpp_str_func_from_defined_type(yaml_type: str):
    if yaml_type == "string_array":
        val_to_cpp_str = str_to_str
    elif yaml_type == "double_array":
        val_to_cpp_str = float_to_str
    elif yaml_type == "integer_array":
        val_to_cpp_str = int_to_str
    elif yaml_type == "bool_array":
        val_to_cpp_str = bool_to_str
    elif yaml_type == "string":
        val_to_cpp_str = str_to_str
    elif yaml_type.__contains__("string_fixed_") and is_fixed_type(yaml_type):
        val_to_cpp_str = fixed_str_to_str
    elif yaml_type == "double":
        val_to_cpp_str = float_to_str
    elif yaml_type == "int":
        val_to_cpp_str = int_to_str
    elif yaml_type == "bool":
        val_to_cpp_str = bool_to_str
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

    return val_to_cpp_str


# TODO type checking fails here
# @typechecked
def cpp_str_func_from_python_val(arg):
    if isinstance(arg, int):
        val_func = int_to_str
    elif isinstance(arg, float):
        val_func = float_to_str
    elif isinstance(arg, bool):
        val_func = bool_to_str
    elif isinstance(arg, str):
        val_func = str_to_str
    elif arg is None:
        val_func = lambda x: ""
    else:
        raise compile_error("invalid python arg type: %s" % type(arg))
    return val_func


@typechecked
def get_parameter_as_function_str(yaml_type: str) -> str:
    if yaml_type == "string_array":
        parameter_conversion = "as_string_array()"
    elif yaml_type == "double_array":
        parameter_conversion = "as_double_array()"
    elif yaml_type == "int_array":
        parameter_conversion = "as_integer_array()"
    elif yaml_type == "bool_array":
        parameter_conversion = "as_bool_array()"
    elif yaml_type == "string" or yaml_type.__contains__("string_fixed_"):
        parameter_conversion = "as_string()"
    elif yaml_type == "double":
        parameter_conversion = "as_double()"
    elif yaml_type == "int":
        parameter_conversion = "as_int()"
    elif yaml_type == "bool":
        parameter_conversion = "as_bool()"
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

    return parameter_conversion


@typechecked
def pascal_case(string: str):
    words = string.split("_")
    return "".join(w.title() for w in words)


@typechecked
def initialization_fail_validation(param_name: str) -> str:
    return (
        f"throw rclcpp::exceptions::InvalidParameterValueException"
        f'(fmt::format("Invalid value set during initialization for '
        f"parameter '{param_name}': \" + validation_result.error_msg()));"
    )


@typechecked
def initialization_pass_validation(param_name: str, parameter_conversion: str) -> str:
    return ""


@typechecked
def update_parameter_fail_validation() -> str:
    return "return validation_result;"


@typechecked
def update_parameter_pass_validation() -> str:
    return ""


def get_dynamic_parameter_field(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    parameter_field = tmp[-1]
    return parameter_field


def get_dynamic_mapped_parameter(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    tmp2 = tmp[-2].split("_")
    mapped_param = tmp2[-1]
    return mapped_param


def get_dynamic_struct_name(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    struct_name = tmp[:-2]
    return ".".join(struct_name)


def get_dynamic_parameter_name(yaml_parameter_name: str):
    struct_name = get_dynamic_struct_name(yaml_parameter_name)
    parameter_field = get_dynamic_parameter_field(yaml_parameter_name)
    parameter_name = [struct_name, parameter_field]
    parameter_name = ".".join(parameter_name)
    return parameter_name


def get_dynamic_parameter_map(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    parameter_map = tmp[:-2]
    mapped_param = get_dynamic_mapped_parameter(yaml_parameter_name)
    parameter_map.append(mapped_param + "_map")
    parameter_map = ".".join(parameter_map)
    return parameter_map


def get_parameter_type(yaml_type: str):
    if is_fixed_type(yaml_type):
        return get_fixed_type(yaml_type).upper()
    else:
        return yaml_type.upper()


# Each template has a corresponding class with the str filling in the template with jinja


class DeclareParameter:
    @typechecked
    def __init__(
        self,
        parameter_name: str,
        parameter_description: str,
        parameter_read_only: bool,
        yaml_type: str,
        has_default_value: any,
    ):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = get_parameter_type(yaml_type)
        self.has_default_value = has_default_value

    def __str__(self):
        if self.has_default_value:
            default_value = "not_empty"
        else:
            default_value = ""

        data = {
            "parameter_name": self.parameter_name,
            "parameter_value": self.parameter_name,
            "parameter_type": self.parameter_type,
            "parameter_description": self.parameter_description,
            "parameter_read_only": bool_to_str(self.parameter_read_only),
            "default_value": default_value,
        }

        j2_template = Template(GenerateCode.templates["declare_parameter"])
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
            declare_str = f"{type_str} {self.variable_name};\n"
        elif isinstance(self.value, list):
            value_str = "{"
            value_str += ", ".join(val_func(x) for x in self.value)
            value_str += "}"
            declare_str = f"{type_str} {self.variable_name} = {value_str};\n"
        else:
            value_str = val_func(self.value)
            declare_str = f"{type_str} {self.variable_name} = {value_str};\n"
        return declare_str


class Struct:
    @typechecked
    def __init__(self, struct_name: str, fields: list[VariableDeclaration]):
        self.struct_name = struct_name
        self.fields = fields
        self.sub_structs = []
        self.struct_instance = ""

    @typechecked
    def add_field(self, field: VariableDeclaration):
        self.fields.append(field)

    def add_sub_struct(self, sub_struct):
        self.sub_structs.append(sub_struct)

    def inner_content(self):
        content = "".join(str(x) for x in self.fields)
        content += "".join(str(x) for x in self.sub_structs)

        return str(content)

    def __str__(self):
        sub_struct_str = "".join(str(x) for x in self.sub_structs)
        field_str = "".join(str(x) for x in self.fields)

        if is_mapped_parameter(self.struct_name):
            map_val_type = pascal_case(self.struct_name)
            map_name = self.struct_name.split("_")[-1] + "_map"
            map_name = map_name.replace(".", "_")
        else:
            map_val_type = ""
            map_name = ""
            self.struct_instance = self.struct_name

        data = {
            "struct_name": pascal_case(self.struct_name),
            "struct_instance": self.struct_instance,
            "struct_fields": str(field_str),
            "sub_structs": str(sub_struct_str),
            "map_value_type": map_val_type,
            "map_name": map_name,
        }

        j2_template = Template(GenerateCode.templates["declare_struct"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class ValidationFunction:
    @typechecked
    def __init__(
        self, function_name: str, arguments: Optional[list[any]], defined_type: str
    ):
        self.function_name = function_name
        if function_name[-2:] == "<>":
            self.function_name = function_name[:-2]
            template_type = cpp_primitive_type_from_defined_type(defined_type)
            self.function_name += f"<{template_type}>"

        if arguments is not None:
            self.arguments = arguments
        else:
            self.arguments = []

    def __str__(self):
        code = self.function_name + "(param"
        for arg in self.arguments:
            if isinstance(arg, list):
                code += ", {"
                for a in arg[:-1]:
                    val_func = cpp_str_func_from_python_val(a)
                    code += val_func(a) + ", "
                val_func = cpp_str_func_from_python_val(arg[-1])
                code += val_func(arg[-1])
                code += "}"
            else:
                val_func = cpp_str_func_from_python_val(arg)
                code += ", " + val_func(arg)
        code += ")"

        return code


class ParameterValidation:
    @typechecked
    def __init__(
        self,
        invalid_effect: str,
        valid_effect: str,
        validation_function: ValidationFunction,
    ):
        self.invalid_effect = invalid_effect
        self.valid_effect = valid_effect
        self.validation_function = validation_function

    def __str__(self):
        data = {
            "validation_function": str(self.validation_function),
            "valid_effect": self.valid_effect,
            "invalid_effect": self.invalid_effect,
        }

        j2_template = Template(GenerateCode.templates["parameter_validation"])
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
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)

        data = {
            "parameter_name": self.parameter_name,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenerateCode.templates["update_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DynamicUpdateParameter:
    @typechecked
    def __init__(self, parameter_name: str, parameter_as_function: str):
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []

    @typechecked
    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)

        mapped_param = get_dynamic_mapped_parameter(self.parameter_name)
        parameter_map = get_dynamic_parameter_map(self.parameter_name)
        parameter_name = get_dynamic_parameter_name(self.parameter_name)
        struct_name = get_dynamic_struct_name(self.parameter_name)
        parameter_field = get_dynamic_parameter_field(self.parameter_name)

        data = {
            "mapped_param": mapped_param,
            "parameter_map": parameter_map,
            "struct_name": struct_name,
            "parameter_field": parameter_field,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenerateCode.templates["dynamic_update_parameter"])
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
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)

        data = {
            "parameter_name": self.parameter_name,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenerateCode.templates["declare_parameter_set"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DynamicDeclareParameter:
    @typechecked
    def __init__(
        self,
        parameter_name: str,
        parameter_description: str,
        parameter_read_only: bool,
        yaml_type: str,
        has_default_value: any,
        parameter_as_function: str,
    ):
        self.parameter_name = parameter_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_type = get_parameter_type(yaml_type)
        self.has_default_value = has_default_value
        self.parameter_name = parameter_name
        self.parameter_as_function = parameter_as_function
        self.parameter_validations = []
        self.param_struct_instance = "params_"

    @typechecked
    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)

    def __str__(self):
        if self.has_default_value:
            default_value = "not_empty"
        else:
            default_value = ""

        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)

        mapped_param = get_dynamic_mapped_parameter(self.parameter_name)
        parameter_map = get_dynamic_parameter_map(self.parameter_name)
        struct_name = get_dynamic_struct_name(self.parameter_name)
        parameter_field = get_dynamic_parameter_field(self.parameter_name)

        data = {
            "struct_name": struct_name,
            "parameter_type": self.parameter_type,
            "parameter_description": self.parameter_description,
            "parameter_read_only": bool_to_str(self.parameter_read_only),
            "default_value": default_value,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
            "mapped_param": mapped_param,
            "mapped_param_underscore": mapped_param.replace(".", "_"),
            "parameter_field": parameter_field,
            "parameter_map": parameter_map,
            "param_struct_instance": self.param_struct_instance,
        }

        j2_template = Template(GenerateCode.templates["dynamic_declare_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class RemoveDynamicParameter:
    @typechecked
    def __init__(self, dynamic_declare_parameter: DynamicDeclareParameter):
        self.dynamic_declare_parameter = dynamic_declare_parameter

    def __str__(self):
        parameter_map = get_dynamic_parameter_map(
            self.dynamic_declare_parameter.parameter_name
        )
        struct_name = get_dynamic_struct_name(
            self.dynamic_declare_parameter.parameter_name
        )
        parameter_field = get_dynamic_parameter_field(
            self.dynamic_declare_parameter.parameter_name
        )
        mapped_param = get_dynamic_mapped_parameter(
            self.dynamic_declare_parameter.parameter_name
        )

        data = {
            "parameter_map": parameter_map,
            "mapped_param": mapped_param,
            "dynamic_declare_parameter": str(self.dynamic_declare_parameter),
            "struct_name": struct_name,
            "parameter_field": parameter_field,
        }

        j2_template = Template(GenerateCode.templates["remove_dynamic_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


def get_all_templates():
    template_path = os.path.join(os.path.dirname(__file__), "jinja_templates")
    template_map = {}
    for file_name in [
        f
        for f in os.listdir(template_path)
        if os.path.isfile(os.path.join(template_path, f))
    ]:
        with open(os.path.join(template_path, file_name)) as file:
            template_map[file_name] = file.read()

    return template_map


# class used to generate c++ code from yaml file
class GenerateCode:
    templates = get_all_templates()

    def __init__(self):
        self.namespace = ""
        self.struct_tree = Struct("Params", [])
        self.update_parameters = []
        self.declare_parameters = []
        self.declare_dynamic_parameters = []
        self.update_dynamic_parameters = []
        self.update_declare_dynamic_parameter = []
        self.remove_dynamic_parameter = []
        self.declare_parameter_sets = []
        self.comments = "// auto-generated DO NOT EDIT"
        self.user_validation_file = ""

    def preprocess_inputs(self, name, value, nested_name_list):
        # define parameter name
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        # required attributes
        try:
            defined_type = value["type"]
        except KeyError as e:
            raise compile_error("No type defined for parameter %s" % param_name)

        # optional attributes
        default_value = value.get("default_value", None)
        description = value.get("description", "")
        read_only = bool(value.get("read_only", False))
        validations = []
        validations_dict = value.get("validation", {})
        if is_fixed_type(defined_type):
            validations_dict["size_lt<>"] = fixed_type_size(defined_type) + 1

        for func_name in validations_dict:
            args = validations_dict[func_name]
            if args is not None and not isinstance(args, list):
                args = [args]
            validations.append(ValidationFunction(func_name, args, defined_type))

        return (
            param_name,
            defined_type,
            default_value,
            description,
            read_only,
            validations,
        )

    def parse_dynamic_params(self, name, value, nested_name_list):

        (
            param_name,
            defined_type,
            default_value,
            description,
            read_only,
            validations,
        ) = self.preprocess_inputs(name, value, nested_name_list)

        # define struct
        var = VariableDeclaration(defined_type, name, default_value)
        self.struct_tree.add_field(var)

        # declare and set parameter
        parameter_conversion = get_parameter_as_function_str(defined_type)
        declare_parameter_invalid = initialization_fail_validation(param_name)
        declare_parameter_valid = initialization_pass_validation(
            param_name, parameter_conversion
        )
        dynamic_declare_parameter = DynamicDeclareParameter(
            param_name,
            description,
            read_only,
            defined_type,
            default_value is not None,
            parameter_conversion,
        )
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                declare_parameter_invalid, declare_parameter_valid, validation_function
            )
            dynamic_declare_parameter.add_parameter_validation(parameter_validation)

        self.declare_dynamic_parameters.append(dynamic_declare_parameter)

        # remove destroyed parameters
        dynamic_update_parameter = RemoveDynamicParameter(dynamic_declare_parameter)
        self.remove_dynamic_parameter.append(dynamic_update_parameter)

        # declare new dynamic parameters
        dynamic_declare_parameter = DynamicDeclareParameter(
            param_name,
            description,
            read_only,
            defined_type,
            default_value is not None,
            parameter_conversion,
        )
        self.update_declare_dynamic_parameter.append(dynamic_declare_parameter)

        # update dynamic parameter
        update_parameter_invalid = update_parameter_fail_validation()
        update_parameter_valid = update_parameter_pass_validation()
        parameter_conversion = get_parameter_as_function_str(defined_type)
        update_parameter = DynamicUpdateParameter(param_name, parameter_conversion)
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                update_parameter_invalid, update_parameter_valid, validation_function
            )
            update_parameter.add_parameter_validation(parameter_validation)

        self.update_dynamic_parameters.append(update_parameter)

    def parse_params(self, name, value, nested_name_list):

        (
            param_name,
            defined_type,
            default_value,
            description,
            read_only,
            validations,
        ) = self.preprocess_inputs(name, value, nested_name_list)

        # define struct
        var = VariableDeclaration(defined_type, name, default_value)
        self.struct_tree.add_field(var)

        # declare parameter
        declare_parameter = DeclareParameter(
            param_name, description, read_only, defined_type, default_value is not None
        )
        self.declare_parameters.append(declare_parameter)

        # update parameter
        update_parameter_invalid = update_parameter_fail_validation()
        update_parameter_valid = update_parameter_pass_validation()
        parameter_conversion = get_parameter_as_function_str(defined_type)
        update_parameter = UpdateParameter(param_name, parameter_conversion)
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                update_parameter_invalid, update_parameter_valid, validation_function
            )
            update_parameter.add_parameter_validation(parameter_validation)

        self.update_parameters.append(update_parameter)

        # set parameter
        declare_parameter_invalid = initialization_fail_validation(param_name)
        declare_parameter_valid = initialization_pass_validation(
            param_name, parameter_conversion
        )
        declare_parameter_set = DeclareParameterSet(param_name, parameter_conversion)
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                declare_parameter_invalid, declare_parameter_valid, validation_function
            )
            declare_parameter_set.add_parameter_validation(parameter_validation)

        self.declare_parameter_sets.append(declare_parameter_set)

    def parse_dict(self, name, root_map, nested_name):

        if isinstance(root_map, dict) and isinstance(
            next(iter(root_map.values())), dict
        ):
            cur_struct_tree = self.struct_tree

            sub_struct = Struct(name, [])
            self.struct_tree.add_sub_struct(sub_struct)
            self.struct_tree = sub_struct
            for key in root_map:
                if isinstance(root_map[key], dict):
                    nested_name.append(name)
                    self.parse_dict(key, root_map[key], nested_name)
                    nested_name.pop()

            self.struct_tree = cur_struct_tree
        else:
            if is_mapped_parameter(self.struct_tree.struct_name):
                self.parse_dynamic_params(name, root_map, nested_name)
            else:
                self.parse_params(name, root_map, nested_name)

    def __str__(self):
        data = {
            "user_validation_file": self.user_validation_file,
            "comments": self.comments,
            "namespace": self.namespace,
            "struct_content": self.struct_tree.sub_structs[0].inner_content(),
            "update_params_set": "\n".join([str(x) for x in self.update_parameters]),
            "update_dynamic_parameters": "\n".join(
                [str(x) for x in self.update_dynamic_parameters]
            ),
            "declare_params": "\n".join([str(x) for x in self.declare_parameters]),
            "declare_params_set": "\n".join(
                [str(x) for x in self.declare_parameter_sets]
            ),
            "declare_set_dynamic_params": "\n".join(
                [str(x) for x in self.declare_dynamic_parameters]
            ),
            "update_declare_dynamic_parameters": "\n".join(
                [str(x) for x in self.update_declare_dynamic_parameter]
            ),
            # TODO support removing runtime parameters
            # "remove_dynamic_parameters": "\n".join(
            #     [str(x) for x in self.remove_dynamic_parameter]
            # ),
        }

        j2_template = Template(GenerateCode.templates["parameter_listener"])
        code = j2_template.render(data, trim_blocks=True)
        return code

    def run(self):
        if 3 > len(sys.argv) > 4:
            raise compile_error(
                "generate_parameter_library_py expects three input argument: output_file, "
                "yaml file path, [validate include header]"
            )

        param_gen_directory = sys.argv[0].split("/")
        param_gen_directory = "".join(x + "/" for x in param_gen_directory[:-1])
        if param_gen_directory[-1] != "/":
            param_gen_directory += "/"

        output_file = sys.argv[1]
        output_dir = os.path.dirname(output_file)
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        yaml_file = sys.argv[2]
        with open(yaml_file) as f:
            try:
                docs = yaml.load_all(f, Loader=yaml.FullLoader)
                doc = list(docs)[0]
            except ParserError as e:
                raise compile_error(str(e))

            if len(doc) != 1:
                raise compile_error(
                    "The yaml definition must only have one root element"
                )
            self.namespace = list(doc.keys())[0]
            self.parse_dict(self.namespace, doc[self.namespace], [])

        if len(sys.argv) > 3:
            self.user_validation_file = sys.argv[3]

        code = str(self)
        with open(output_file, "w") as f:
            f.write(code)


def main():
    gen_param_struct = GenerateCode()
    gen_param_struct.run()


if __name__ == "__main__":
    sys.exit(main())
