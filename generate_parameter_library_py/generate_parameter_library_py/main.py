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
from typing import Callable, Optional
from typeguard import typechecked
from jinja2 import Template


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
    elif yaml_type == "double":
        cpp_type = "double"
    elif yaml_type == "integer":
        cpp_type = "int"
    elif yaml_type == "bool":
        cpp_type = "bool"
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

    return cpp_type


# cpp_type, val_to_cpp_str, parameter_conversion
@typechecked
def cpp_str_func_from_defined_type(yaml_type: str) -> Callable:
    if yaml_type == "string_array":
        val_to_cpp_str = str_to_str
    elif yaml_type == "double_array":
        val_to_cpp_str = float_to_str
    elif yaml_type == "int_array":
        val_to_cpp_str = int_to_str
    elif yaml_type == "bool_array":
        val_to_cpp_str = bool_to_str
    elif yaml_type == "string":
        val_to_cpp_str = str_to_str
    elif yaml_type == "double":
        val_to_cpp_str = float_to_str
    elif yaml_type == "integer":
        val_to_cpp_str = int_to_str
    elif yaml_type == "bool":
        val_to_cpp_str = bool_to_str
    else:
        raise compile_error("invalid yaml type: %s" % type(yaml_type))

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
    elif yaml_type == "string":
        parameter_conversion = "as_string()"
    elif yaml_type == "double":
        parameter_conversion = "as_double()"
    elif yaml_type == "integer":
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
        f'throw rclcpp::exceptions::InvalidParameterValueException("Invalid value set during initialization for '
        f'parameter {param_name}: + validation_result.error_msg());"); '
    )


@typechecked
def initialization_pass_validation(param_name: str, parameter_conversion: str) -> str:
    return ""  # f"params_.{param_name} = param.{parameter_conversion};"


@typechecked
def update_parameter_fail_validation() -> str:
    return "result.successful = false;\nbreak;"


@typechecked
def update_parameter_pass_validation() -> str:
    return ""


# Each template has a corresponding class with the str filling in the template with jinja


class DeclareParameter:
    @typechecked
    def __init__(
        self,
        parameter_name: str,
        parameter_description: str,
        parameter_read_only: bool,
        parameter_type: str,
        default_value: any,
    ):
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

        data = {
            "parameter_name": self.parameter_name,
            "parameter_type": parameter_type,
            "parameter_description": self.parameter_description,
            "parameter_read_only": bool_to_str(self.parameter_read_only),
            "default_value": default_value,
        }

        j2_template = Template(GenParamStruct.templates["declare_parameter"])
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
            value_str = "{"
            for val in self.value[:-1]:
                value_str += val_func(val) + ", "
            if len(self.value) > 0:
                value_str += val_func(self.value[-1])
            value_str += "}"

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

    def add_sub_struct(self, sub_struct):
        self.sub_structs.append(sub_struct)

    def inner_content(self):
        content = Buffer()
        for field in self.fields:
            content += str(field) + "\n"
        for sub_struct in self.sub_structs:
            content += str(sub_struct) + "\n"

        return str(content)

    def __str__(self):
        sub_struct_str = Buffer()
        for sub_struct in self.sub_structs:
            sub_struct_str += str(sub_struct) + "\n"

        field_str = Buffer()
        for field in self.fields:
            field_str += str(field) + "\n"

        data = {
            "struct_name": pascal_case(self.struct_name),
            "struct_instance": self.struct_name,
            "struct_fields": str(field_str),
            "sub_structs": str(sub_struct_str),
        }

        j2_template = Template(GenParamStruct.templates["declare_struct"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class ValidationFunction:
    @typechecked
    def __init__(self, function_name: str, arguments: list[any]):
        # if isinstance(arguments, list) and isinstance(arguments[0], list):
        #     pass
        self.function_name = function_name
        self.arguments = arguments

    def __str__(self):
        code = self.function_name + "(param"
        for arg in self.arguments:
            if isinstance(arg, list):
                code += ", {"
                for ind, a in enumerate(arg):
                    val_func = cpp_str_func_from_python_val(a)
                    delim = ", "
                    if ind == len(arg) - 1:
                        delim = ""
                    code += val_func(a) + delim
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

        j2_template = Template(GenParamStruct.templates["parameter_validation"])
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

        data = {
            "parameter_name": self.parameter_name,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenParamStruct.templates["update_parameter"])
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
            parameter_validations_str += str(parameter_validation) + "\n"

        data = {
            "parameter_name": self.parameter_name,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenParamStruct.templates["declare_parameter_set"])
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


# class used to fill template text file with passed in yaml file
class GenParamStruct:
    templates = get_all_templates()

    def __init__(self):
        self.namespace = ""
        self.struct_tree = Struct("Params", [])
        self.update_parameters = []
        self.declare_parameters = []
        self.declare_parameter_sets = []
        self.comments = "// this is auto-generated code "
        self.user_validation = ""
        self.validation_functions = ""

    def parse_params(self, name, value, nested_name_list):
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
        bounds = value.get("bounds", None)
        fixed_size = value.get("fixed_size", None)
        one_of = value.get("one_of", None)
        validations = value.get("validation", [])

        # # validate inputs
        if bounds is not None and not validate_type(defined_type, bounds):
            raise compile_error(
                "The type of the bounds must be the same type as the defined type"
            )
        if bounds is not None and len(bounds) != 2:
            raise compile_error("The bounds must have two inputs: [lower, upper]")
        if default_value and not validate_type(defined_type, default_value):
            raise compile_error(
                "The type of the default_value must be the same type as the defined type"
            )
        if fixed_size is not None and not isinstance(fixed_size, int):
            raise compile_error(
                "The type of the fixed size attribute must be an integer"
            )
        if one_of is not None and not all(
            validate_type(defined_type, x) for x in one_of
        ):
            raise compile_error("The type of the one_of attribute must be an list")
        if one_of is not None and len(one_of) == 2:
            raise compile_error("The one_of must have at least one input")

        # add default validations if applicable
        if len(validations) > 0 and isinstance(validations[0], str):
            validations = [validations]

        if bounds is not None:
            validations.append(
                ["validate_" + defined_type + "_bounds", bounds[0], bounds[1]]
            )

        if fixed_size is not None:
            validations.append(["validate_" + defined_type + "_len", fixed_size])

        if one_of is not None:
            validations.append(["validate_" + defined_type + "_one_of", one_of])

        # define struct
        var = VariableDeclaration(defined_type, name, default_value)
        self.struct_tree.add_field(var)

        # declare parameter
        declare_parameter = DeclareParameter(
            param_name, description, read_only, defined_type, default_value
        )
        self.declare_parameters.append(declare_parameter)

        # update parameter
        update_parameter_invalid = update_parameter_fail_validation()
        update_parameter_valid = update_parameter_pass_validation()
        parameter_conversion = get_parameter_as_function_str(defined_type)
        update_parameter = UpdateParameter(param_name, parameter_conversion)
        for validation in validations:
            validation_function = ValidationFunction(validation[0], validation[1:])
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
        for validation in validations:
            validation_function = ValidationFunction(validation[0], validation[1:])
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

            if name != self.namespace:
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
            self.parse_params(name, root_map, nested_name)

    def __str__(self):
        data = {
            "USER_VALIDATORS": self.user_validation,
            "COMMENTS": self.comments,
            "namespace": self.namespace,
            "validation_functions": self.validation_functions,
            "struct_content": self.struct_tree.inner_content(),
            "update_params_set": "\n".join([str(x) for x in self.update_parameters]),
            "declare_params": "\n".join([str(x) for x in self.declare_parameters]),
            "declare_params_set": "\n".join(
                [str(x) for x in self.declare_parameter_sets]
            ),
        }

        j2_template = Template(GenParamStruct.templates["parameter_listener"])
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
                    "the controller yaml definition must only have one root element"
                )
            self.namespace = list(doc.keys())[0]
            self.parse_dict(self.namespace, doc[self.namespace], [])

        if len(sys.argv) > 3:
            user_validation_file = sys.argv[3]
            with open(user_validation_file, "r") as f:
                self.user_validation = f.read()

        validation_functions_file = os.path.join(
            os.path.dirname(__file__), "cpp_templates", "validators.hpp"
        )
        with open(validation_functions_file, "r") as f:
            self.validation_functions = f.read()

        code = str(self)
        with open(output_file, "w") as f:
            f.write(code)


def main():
    gen_param_struct = GenParamStruct()
    gen_param_struct.run()
    pass


if __name__ == "__main__":
    sys.exit(main())
