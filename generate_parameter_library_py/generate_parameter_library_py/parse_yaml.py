#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2023 PickNik Inc.
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

from jinja2 import Template
from typeguard import typechecked
from typing import List, Optional
from yaml.parser import ParserError
from yaml.scanner import ScannerError
import os
import yaml

from generate_parameter_library_py.cpp_convertions import CPPConverstions
from generate_parameter_library_py.python_convertions import PythonConvertions


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
def pascal_case(string: str):
    words = string.split("_")
    return "".join(w.title() for w in words)


@typechecked
def int_to_integer_str(value: str):
    return value.replace("int", "integer")


def get_dynamic_parameter_field(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    parameter_field = tmp[-1]
    return parameter_field


def get_dynamic_mapped_parameter(yaml_parameter_name: str):
    tmp = yaml_parameter_name.split(".")
    tmp2 = tmp[-2]
    mapped_param = tmp2.replace("__map_", "")
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


@typechecked
def is_fixed_type(yaml_type: str):
    return fixed_type_size(yaml_type) is not None


@typechecked
def get_fixed_base_type(yaml_type: str):
    tmp = yaml_type.split("_")
    return "_".join(tmp[: -(min(2, len(tmp) - 1))])


@typechecked
def get_fixed_type(yaml_type: str):
    return get_fixed_base_type(yaml_type) + "_fixed"


class CodeGenVariableBase:
    @typechecked
    def __init__(
        self,
        language: str,
        name: str,
        param_name: str,
        defined_type: str,
        default_value: any,
    ):
        if language == "cpp":
            self.conversation = CPPConverstions()
        elif language == "rst" or language == "markdown":
            # cpp is used here because it the desired style of the markdown,
            # e.g. "false" for C++ instead of "False" for Python
            self.conversation = CPPConverstions()
        elif language == "python":
            self.conversation = PythonConvertions()
        else:
            raise compile_error(
                "Invalid language, only c++ and python are currently supported."
            )

        self.name = name
        self.default_value = default_value
        self.name = name
        self.param_name = param_name
        self.defined_type, template = self.process_type(defined_type)
        self.array_type = array_type(self.defined_type)

        if self.defined_type not in self.conversation.defined_type_to_lang_type:
            allowed = ", ".join(
                key for key in self.conversation.defined_type_to_lang_type
            )
            raise compile_error(
                f"Invalid parameter type `{defined_type}` for parameter {param_name}. Allowed types are: "
                + allowed
            )
        func = self.conversation.defined_type_to_lang_type[self.defined_type]
        self.lang_type = func(self.defined_type, template)
        tmp = defined_type.split("_")
        self.defined_base_type = tmp[0]
        func = self.conversation.defined_type_to_lang_type[self.defined_base_type]
        self.lang_base_type = func(self.defined_base_type, template)
        func = self.conversation.lang_str_value_func[self.defined_type]
        try:
            self.lang_str_value = func(default_value)
        except TypeError:
            raise compile_error(
                f"Parameter {param_name} has incorrect type. Expected: {defined_type}, got: {self.get_yaml_type_from_python(default_value)}"
            )

    def parameter_as_function_str(self):
        if self.defined_type not in self.conversation.yaml_type_to_as_function:
            raise compile_error("invalid yaml type: %s" % type(self.defined_type))
        return self.conversation.yaml_type_to_as_function[self.defined_type]

    def get_python_val_to_str_func(self, arg):
        return self.conversation.python_val_to_str_func[str(type(arg))]

    def get_yaml_type_from_python(self, arg):
        if isinstance(arg, list):
            return self.conversation.python_list_to_yaml_type[str(type(arg[0]))]
        else:
            return self.conversation.python_val_to_yaml_type[str(type(arg[0]))]

    def process_type(self, defined_type):
        raise NotImplemented()

    def get_parameter_type(self):
        raise NotImplemented()


class CodeGenVariable(CodeGenVariableBase):
    def process_type(self, defined_type):
        return defined_type, None

    def get_parameter_type(self):
        return int_to_integer_str(self.defined_type).upper()


class CodeGenFixedVariable(CodeGenVariableBase):
    def process_type(self, defined_type):
        size = fixed_type_size(defined_type)
        tmp = defined_type.split("_")
        yaml_base_type = tmp[0]
        func = self.conversation.defined_type_to_lang_type[yaml_base_type]
        lang_base_type = func(yaml_base_type, None)
        defined_type = get_fixed_type(defined_type)
        return defined_type, (lang_base_type, size)

    def get_parameter_type(self):
        return int_to_integer_str(get_fixed_base_type(self.defined_type)).upper()


# Each template has a corresponding class with the str filling in the template with jinja
class VariableDeclaration:
    @typechecked
    def __init__(self, code_gen_variable: CodeGenVariableBase):
        self.code_gen_variable = code_gen_variable

    def __str__(self):
        value = self.code_gen_variable.lang_str_value
        data = {
            "type": self.code_gen_variable.lang_type,
            "name": self.code_gen_variable.name,
            "value": value,
        }

        j2_template = Template(GenerateCode.templates["declare_variable"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DeclareStruct:
    @typechecked
    def __init__(self, struct_name: str, fields: List[VariableDeclaration]):
        self.struct_name = struct_name
        self.fields = fields
        self.sub_structs = []
        self.struct_instance = ""

    @typechecked
    def add_field(self, field: VariableDeclaration):
        self.fields.append(field)

    def add_sub_struct(self, sub_struct):
        self.sub_structs.append(sub_struct)

    def field_content(self):
        content = "".join(str(x) for x in self.fields)
        return str(content)

    def sub_struct_content(self):
        content = "".join(str(x) for x in self.sub_structs)
        return str(content)

    def __str__(self):
        sub_struct_str = "".join(str(x) for x in self.sub_structs)
        field_str = "".join(str(x) for x in self.fields)
        if field_str == "" and sub_struct_str == "":
            return ""

        if is_mapped_parameter(self.struct_name):
            map_val_type = pascal_case(self.struct_name)
            map_name = self.struct_name.replace("__map_", "") + "_map"
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
        self,
        function_name: str,
        arguments: Optional[List[any]],
        code_gen_variable: CodeGenVariableBase,
    ):
        self.code_gen_variable = code_gen_variable
        self.function_name = function_name
        self.function_base_name = function_name.replace("<>", "")

        if arguments is not None:
            self.arguments = arguments
        else:
            self.arguments = []

    def __str__(self):
        function_name = self.code_gen_variable.conversation.get_func_signature(
            self.function_name, self.code_gen_variable.lang_base_type
        )
        open_bracket = self.code_gen_variable.conversation.open_bracket
        close_bracket = self.code_gen_variable.conversation.close_bracket

        code = function_name + "(param"
        for arg in self.arguments:
            if isinstance(arg, list):
                code += ", " + open_bracket
                for a in arg[:-1]:
                    val_func = self.code_gen_variable.get_python_val_to_str_func(a)
                    code += val_func(a) + ", "
                val_func = self.code_gen_variable.get_python_val_to_str_func(arg[-1])
                code += val_func(arg[-1])
                code += close_bracket
            else:
                val_func = self.code_gen_variable.get_python_val_to_str_func(arg)
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


class UpdateParameterBase:
    @typechecked
    def __init__(self, parameter_name: str, code_gen_variable: CodeGenVariableBase):
        self.parameter_name = parameter_name
        self.parameter_as_function = code_gen_variable.parameter_as_function_str()
        self.parameter_validations = []

    @typechecked
    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)


class UpdateParameter(UpdateParameterBase):
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


class UpdateRuntimeParameter(UpdateParameterBase):
    def __str__(self):
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)
        mapped_param = get_dynamic_mapped_parameter(self.parameter_name)
        parameter_map = get_dynamic_parameter_map(self.parameter_name)
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

        j2_template = Template(GenerateCode.templates["update_runtime_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class SetStackParams:
    @typechecked
    def __init__(self, parameter_name: str):
        self.parameter_name = parameter_name

    def __str__(self):
        data = {"parameter_name": self.parameter_name}
        j2_template = Template(GenerateCode.templates["set_stack_params"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class SetParameterBase:
    @typechecked
    def __init__(self, parameter_name: str, code_gen_variable: CodeGenVariableBase):
        self.parameter_name = parameter_name
        self.parameter_as_function = code_gen_variable.parameter_as_function_str()
        self.parameter_validations = []

    @typechecked
    def add_parameter_validation(self, parameter_validation: ParameterValidation):
        self.parameter_validations.append(parameter_validation)


class SetParameter(SetParameterBase):
    def __str__(self):
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)

        data = {
            "parameter_name": self.parameter_name,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenerateCode.templates["set_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class SetRuntimeParameter(SetParameterBase):
    def __str__(self):
        parameter_validations_str = "".join(str(x) for x in self.parameter_validations)
        parameter_field = get_dynamic_parameter_field(self.parameter_name)
        data = {
            "parameter_field": parameter_field,
            "parameter_validations": str(parameter_validations_str),
            "parameter_as_function": self.parameter_as_function,
        }

        j2_template = Template(GenerateCode.templates["set_runtime_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DeclareParameterBase:
    @typechecked
    def __init__(
        self,
        code_gen_variable: CodeGenVariableBase,
        parameter_description: str,
        parameter_read_only: bool,
        parameter_validations: list,
    ):
        self.parameter_name = code_gen_variable.param_name
        self.parameter_description = parameter_description
        self.parameter_read_only = parameter_read_only
        self.parameter_validations = parameter_validations
        self.code_gen_variable = code_gen_variable


class DeclareParameter(DeclareParameterBase):
    def __str__(self):
        if len(self.code_gen_variable.lang_str_value) == 0:
            self.parameter_value = ""
        else:
            self.parameter_value = self.parameter_name
        bool_to_str = self.code_gen_variable.conversation.bool_to_str

        parameter_validations = self.parameter_validations
        data = {
            "parameter_name": self.parameter_name,
            "parameter_value": self.parameter_value,
            "parameter_type": self.code_gen_variable.get_parameter_type(),
            "parameter_description": self.parameter_description,
            "parameter_read_only": bool_to_str(self.parameter_read_only),
            "parameter_validations": parameter_validations,
        }
        j2_template = Template(GenerateCode.templates["declare_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DeclareRuntimeParameter(DeclareParameterBase):
    def __init__(
        self,
        code_gen_variable: CodeGenVariableBase,
        parameter_description: str,
        parameter_read_only: bool,
        parameter_validations: list,
    ):
        super().__init__(
            code_gen_variable,
            parameter_description,
            parameter_read_only,
            parameter_validations,
        )
        self.set_runtime_parameter = None
        self.param_struct_instance = "updated_params"

    @typechecked
    def add_set_runtime_parameter(self, set_runtime_parameter: SetRuntimeParameter):
        self.set_runtime_parameter = set_runtime_parameter

    def __str__(self):
        if self.set_runtime_parameter is None:
            raise AssertionError(
                "add_set_runtime_parameter was not called before str()"
            )

        if self.code_gen_variable.default_value is None:
            default_value = ""
        else:
            default_value = "non-empty"

        bool_to_str = self.code_gen_variable.conversation.bool_to_str
        parameter_field = get_dynamic_parameter_field(self.parameter_name)
        mapped_param = get_dynamic_mapped_parameter(self.parameter_name)
        parameter_map = get_dynamic_parameter_map(self.parameter_name)
        struct_name = get_dynamic_struct_name(self.parameter_name)

        data = {
            "struct_name": struct_name,
            "parameter_type": self.code_gen_variable.get_parameter_type(),
            "parameter_description": self.parameter_description,
            "parameter_read_only": bool_to_str(self.parameter_read_only),
            "parameter_as_function": self.code_gen_variable.parameter_as_function_str(),
            "mapped_param": mapped_param,
            "mapped_param_underscore": mapped_param.replace(".", "_"),
            "set_runtime_parameter": self.set_runtime_parameter,
            "parameter_map": parameter_map,
            "param_struct_instance": self.param_struct_instance,
            "parameter_field": parameter_field,
            "default_value": default_value,
            "parameter_validations": self.parameter_validations,
        }

        j2_template = Template(GenerateCode.templates["declare_runtime_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


class RemoveRuntimeParameter:
    @typechecked
    def __init__(self, dynamic_declare_parameter: DeclareRuntimeParameter):
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

        j2_template = Template(GenerateCode.templates["remove_runtime_parameter"])
        code = j2_template.render(data, trim_blocks=True)
        return code


def get_all_templates(language: str):
    template_lang_path = os.path.join(
        os.path.dirname(__file__), "jinja_templates", language
    )
    if language == "markdown":
        template_markdown_path = os.path.join(
            os.path.dirname(__file__), "jinja_templates", "markdown"
        )
        template_paths = [template_lang_path, template_markdown_path]
    elif language == "rst":
        template_rst_path = os.path.join(
            os.path.dirname(__file__), "jinja_templates", "rst"
        )
        template_paths = [template_lang_path, template_rst_path]
    else:
        template_paths = [template_lang_path]

    template_map = {}
    for template_path in template_paths:
        for file_name in [
            f
            for f in os.listdir(template_path)
            if os.path.isfile(os.path.join(template_path, f))
        ]:
            with open(os.path.join(template_path, file_name)) as file:
                template_map[file_name] = file.read()

    return template_map


def preprocess_inputs(language, name, value, nested_name_list):
    # define parameter name
    param_name = "".join(x + "." for x in nested_name_list[1:]) + name

    # required attributes
    try:
        defined_type = value["type"]
    except KeyError as e:
        raise compile_error("No type defined for parameter %s" % param_name)

    # check for invalid syntax
    valid_keys = {"default_value", "description", "read_only", "validation", "type"}
    invalid_keys = value.keys() - valid_keys
    if len(invalid_keys) > 0:
        raise compile_error(
            "Invalid syntax in parameter %s. '%s' is not valid syntax"
            % (param_name, next(iter(invalid_keys)))
        )

    # optional attributes
    default_value = value.get("default_value", None)
    if not is_fixed_type(defined_type):
        code_gen_variable = CodeGenVariable(
            language, name, param_name, defined_type, default_value
        )
    else:
        code_gen_variable = CodeGenFixedVariable(
            language, name, param_name, defined_type, default_value
        )

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
        validations.append(ValidationFunction(func_name, args, code_gen_variable))

    return (
        code_gen_variable,
        description,
        read_only,
        validations,
    )


# class used to generate c++ code from yaml file
class GenerateCode:
    templates = None

    def __init__(self, language: str):
        GenerateCode.templates = get_all_templates(language)
        self.language = language
        self.namespace = ""
        self.struct_tree = DeclareStruct("Params", [])
        self.stack_struct_tree = DeclareStruct("StackParams", [])
        self.update_parameters = []
        self.declare_parameters = []
        self.declare_dynamic_parameters = []
        self.update_dynamic_parameters = []
        self.update_declare_dynamic_parameter = []
        self.remove_dynamic_parameter = []
        self.declare_parameter_sets = []
        self.set_stack_params = []
        if language == "cpp":
            self.comments = "// auto-generated DO NOT EDIT"
        elif language == "rst":
            self.comments = ".. auto-generated DO NOT EDIT"
        elif language == "python" or language == "markdown":
            self.comments = "# auto-generated DO NOT EDIT"
        else:
            raise compile_error(
                "Invalid language, only c++ and python are currently supported."
            )
        self.user_validation_file = ""

    def parse(self, yaml_file, validate_header):
        with open(yaml_file) as f:
            try:
                docs = yaml.load_all(f, Loader=yaml.Loader)
                doc = list(docs)[0]
            except ParserError as e:
                raise compile_error(str(e))
            except ScannerError as e:
                raise compile_error(str(e))

            if len(doc) != 1:
                raise compile_error(
                    "The yaml definition must only have one root element"
                )
            self.namespace = list(doc.keys())[0]
            self.user_validation_file = validate_header
            self.parse_dict(self.namespace, doc[self.namespace], [])

    def parse_params(self, name, value, nested_name_list):
        (
            code_gen_variable,
            description,
            read_only,
            validations,
        ) = preprocess_inputs(self.language, name, value, nested_name_list)
        # skip accepted params that do not generate code
        if code_gen_variable.lang_type is None:
            return

        param_name = code_gen_variable.param_name
        update_parameter_invalid = (
            code_gen_variable.conversation.update_parameter_fail_validation()
        )
        update_parameter_valid = (
            code_gen_variable.conversation.update_parameter_pass_validation()
        )
        declare_parameter_invalid = (
            code_gen_variable.conversation.initialization_fail_validation(param_name)
        )
        declare_parameter_valid = (
            code_gen_variable.conversation.initialization_pass_validation(param_name)
        )

        # add variable to struct
        var = VariableDeclaration(code_gen_variable)

        # check if runtime parameter
        is_runtime_parameter = is_mapped_parameter(self.struct_tree.struct_name)

        if is_runtime_parameter:
            declare_parameter_set = SetRuntimeParameter(param_name, code_gen_variable)
            declare_parameter = DeclareRuntimeParameter(
                code_gen_variable, description, read_only, validations
            )
            declare_parameter.add_set_runtime_parameter(declare_parameter_set)
            update_parameter = UpdateRuntimeParameter(param_name, code_gen_variable)
        else:
            declare_parameter = DeclareParameter(
                code_gen_variable, description, read_only, validations
            )
            declare_parameter_set = SetParameter(param_name, code_gen_variable)
            update_parameter = UpdateParameter(param_name, code_gen_variable)

        # set parameter
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                declare_parameter_invalid, declare_parameter_valid, validation_function
            )
            declare_parameter_set.add_parameter_validation(parameter_validation)

        # update parameter
        for validation_function in validations:
            parameter_validation = ParameterValidation(
                update_parameter_invalid, update_parameter_valid, validation_function
            )
            update_parameter.add_parameter_validation(parameter_validation)

        self.struct_tree.add_field(var)
        if not is_runtime_parameter and (
            isinstance(code_gen_variable, CodeGenFixedVariable)
            or not (
                code_gen_variable.array_type
                or code_gen_variable.defined_type == "string"
            )
        ):
            self.stack_struct_tree.add_field(var)
            self.set_stack_params.append(SetStackParams(code_gen_variable.param_name))
        if is_runtime_parameter:
            self.declare_dynamic_parameters.append(declare_parameter)
            self.update_dynamic_parameters.append(update_parameter)
            self.update_declare_dynamic_parameter.append(declare_parameter)
            dynamic_update_parameter = RemoveRuntimeParameter(declare_parameter)
            self.remove_dynamic_parameter.append(dynamic_update_parameter)
        else:
            self.declare_parameters.append(declare_parameter)
            self.update_parameters.append(update_parameter)
            self.declare_parameter_sets.append(declare_parameter_set)

    def parse_dict(self, name, root_map, nested_name):
        if isinstance(root_map, dict) and isinstance(
            next(iter(root_map.values())), dict
        ):
            cur_struct_tree = self.struct_tree
            cur_stack_struct_tree = self.stack_struct_tree

            sub_struct = DeclareStruct(name, [])
            sub_stack_struct = DeclareStruct(name, [])
            self.struct_tree.add_sub_struct(sub_struct)
            self.struct_tree = sub_struct
            self.stack_struct_tree.add_sub_struct(sub_stack_struct)
            self.stack_struct_tree = sub_stack_struct
            for key in root_map:
                if isinstance(root_map[key], dict):
                    nested_name.append(name)
                    self.parse_dict(key, root_map[key], nested_name)
                    nested_name.pop()

            self.struct_tree = cur_struct_tree
            self.stack_struct_tree = cur_stack_struct_tree
        else:
            self.parse_params(name, root_map, nested_name)

    def __str__(self):
        data = {
            "user_validation_file": self.user_validation_file,
            "comments": self.comments,
            "namespace": self.namespace,
            "field_content": self.struct_tree.sub_structs[0].field_content(),
            "sub_struct_content": self.struct_tree.sub_structs[0].sub_struct_content(),
            "stack_field_content": self.stack_struct_tree.sub_structs[
                0
            ].field_content(),
            "stack_sub_struct_content": self.stack_struct_tree.sub_structs[
                0
            ].sub_struct_content(),
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
            "set_stack_params": "\n".join([str(x) for x in self.set_stack_params]),
            # TODO support removing runtime parameters
            # "remove_dynamic_parameters": "\n".join(
            #     [str(x) for x in self.remove_dynamic_parameter]
            # ),
        }

        j2_template = Template(GenerateCode.templates["parameter_library_header"])
        code = j2_template.render(data, trim_blocks=True)
        return code
