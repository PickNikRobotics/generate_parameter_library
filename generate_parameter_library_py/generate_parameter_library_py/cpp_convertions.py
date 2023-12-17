# -*- coding: utf-8 -*-
from typing import List, Optional
from jinja2 import Template
from typeguard import typechecked
import os
import yaml
from yaml.parser import ParserError
from yaml.scanner import ScannerError


class CPPConverstions:
    def __init__(self):
        self.defined_type_to_lang_type = {
            'none': lambda defined_type, templates: None,
            'bool': lambda defined_type, templates: 'bool',
            'double': lambda defined_type, templates: 'double',
            'int': lambda defined_type, templates: 'int64_t',
            'string': lambda defined_type, templates: 'std::string',
            'bool_array': lambda defined_type, templates: 'std::vector<bool>',
            'double_array': lambda defined_type, templates: 'std::vector<double>',
            'int_array': lambda defined_type, templates: 'std::vector<int64_t>',
            'string_array': lambda defined_type, templates: 'std::vector<std::string>',
            'double_array_fixed': lambda defined_type, templates: f'rsl::StaticVector<{templates[0]}, {templates[1]}>',
            'int_array_fixed': lambda defined_type, templates: f'rsl::StaticVector<{templates[0]}, {templates[1]}>',
            'string_array_fixed': lambda defined_type, templates: f'rsl::StaticVector<{templates[0]}, {templates[1]}>',
            'string_fixed': lambda defined_type, templates: f'rsl::StaticString<{templates[1]}>',
        }
        self.yaml_type_to_as_function = {
            'none': None,
            'string_array': 'as_string_array()',
            'double_array': 'as_double_array()',
            'int_array': 'as_integer_array()',
            'bool_array': 'as_bool_array()',
            'string': 'as_string()',
            'double': 'as_double()',
            'int': 'as_int()',
            'bool': 'as_bool()',
            'bool_array_fixed': 'as_bool_array()',
            'double_array_fixed': 'as_double_array()',
            'int_array_fixed': 'as_integer_array()',
            'string_array_fixed': 'as_string_array()',
            'string_fixed': 'as_string()',
        }
        self.lang_str_value_func = {
            'none': self.no_code,
            'bool': self.bool_to_str,
            'double': self.float_to_str,
            'int': self.int_to_str,
            'string': self.str_to_str,
            'bool_array': self.bool_array_to_str,
            'double_array': self.float_array_to_str,
            'int_array': self.int_array_to_str,
            'string_array': self.str_array_to_str,
            'bool_array_fixed': self.bool_array_fixed_to_str,
            'double_array_fixed': self.float_array_fixed_to_str,
            'int_array_fixed': self.int_array_fixed_to_str,
            'string_array_fixed': self.str_array_fixed_to_str,
            'string_fixed': self.str_fixed_to_str,
        }
        self.python_val_to_str_func = {
            "<class 'bool'>": self.bool_to_str,
            "<class 'float'>": self.float_to_str,
            "<class 'int'>": self.int_to_str,
            "<class 'str'>": self.str_to_str,
        }
        self.python_val_to_yaml_type = {
            "<class 'bool'>": 'bool',
            "<class 'float'>": 'double',
            "<class 'int'>": 'int',
            "<class 'str'>": 'str',
        }
        self.python_list_to_yaml_type = {
            "<class 'bool'>": 'bool_array',
            "<class 'float'>": 'double_array',
            "<class 'int'>": 'integer_array',
            "<class 'str'>": 'string_array',
        }

        self.open_bracket = '{'
        self.close_bracket = '}'

    @typechecked
    def get_func_signature(self, function_name: str, base_type: str) -> str:
        if function_name[-2:] == '<>':
            function_base_name = function_name[:-2]
            template_type = base_type
            function_name = function_base_name + f'<{template_type}>'
        return function_name

    @typechecked
    def initialization_fail_validation(self, param_name: str) -> str:
        return (
            f'throw rclcpp::exceptions::InvalidParameterValueException'
            f'(fmt::format("Invalid value set during initialization for '
            f"parameter '{param_name}': {{}}\", validation_result.error()));"
        )

    @typechecked
    def initialization_pass_validation(self, param_name: str) -> str:
        return ''

    @typechecked
    def update_parameter_fail_validation(self) -> str:
        return 'return rsl::to_parameter_result_msg(validation_result);'

    @typechecked
    def update_parameter_pass_validation(self) -> str:
        return ''

    @typechecked
    def no_code(self, s: Optional[str]):
        return ''

    # value to c++ string conversion functions
    @typechecked
    def bool_to_str(self, cond: Optional[bool]):
        if cond is None:
            return ''
        return 'true' if cond else 'false'

    @typechecked
    def float_to_str(self, num: Optional[float]):
        if num is None:
            return ''
        str_num = str(num)
        if str_num == 'nan':
            str_num = 'std::numeric_limits<double>::quiet_NaN()'
        elif str_num == 'inf':
            str_num = 'std::numeric_limits<double>::infinity()'
        elif str_num == '-inf':
            str_num = '-std::numeric_limits<double>::infinity()'
        else:
            if len(str_num.split('.')) == 1 and not str_num.__contains__('e'):
                str_num += '.0'

        return str_num

    @typechecked
    def int_to_str(self, num: Optional[int]):
        if num is None:
            return ''
        return str(num)

    @typechecked
    def str_to_str(self, s: Optional[str]):
        if s is None:
            return ''
        return f'"{s}"'

    @typechecked
    def bool_array_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{' + ', '.join(self.bool_to_str(x) for x in values) + '}'

    @typechecked
    def float_array_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{' + ', '.join(self.float_to_str(x) for x in values) + '}'

    @typechecked
    def int_array_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{' + ', '.join(self.int_to_str(x) for x in values) + '}'

    @typechecked
    def str_array_to_str(self, s: Optional[list]):
        if s is None:
            return ''
        return '{' + ', '.join(self.str_to_str(x) for x in s) + '}'

    @typechecked
    def str_array_fixed_to_str(self, s: Optional[list]):
        raise compile_error('not implemented')

    @typechecked
    def str_fixed_to_str(self, s: Optional[str]):
        if s is None:
            return ''
        return '{%s}' % self.str_to_str(s)

    @typechecked
    def float_array_fixed_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{{' + ', '.join(self.float_to_str(x) for x in values) + '}}'

    @typechecked
    def int_array_fixed_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{{' + ', '.join(self.int_to_str(x) for x in values) + '}}'

    @typechecked
    def bool_array_fixed_to_str(self, values: Optional[list]):
        if values is None:
            return ''
        return '{{' + ', '.join(self.bool_to_str(x) for x in values) + '}}'
