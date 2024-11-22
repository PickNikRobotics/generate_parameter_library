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

import argparse
import os
import re
import sys
from jinja2 import Template
from typeguard import typechecked
import yaml

from generate_parameter_library_py.parse_yaml import (
    GenerateCode,
    DeclareParameter,
    DeclareRuntimeParameter,
    ValidationFunction,
)


class ParameterValidationMarkdown:
    @typechecked
    def __init__(self, validation: ValidationFunction):
        self.sentence_conventions = {
            'bounds': 'parameter must be within bounds VALUES',
            'lt': 'less than VALUES',
            'gt': 'greater than VALUES',
            'lt_eq': 'less than or equal to VALUES',
            'gt_eq': 'greater than or equal to VALUES',
            'one_of': 'one of the specified values: VALUES',
            'fixed_size': 'length must be equal to VALUES',
            'size_gt': 'length is greater than VALUES',
            'size_lt': 'length is less than VALUES',
            'not_empty': 'parameter is not empty',
            'unique': 'contains no duplicates',
            'subset_of': 'every element is one of the list VALUES',
            'element_bounds': 'each element of array must be within bounds VALUES',
            'lower_element_bounds': 'each element of array must be greater than or equal to VALUES',
            'upper_element_bounds': 'each element of array must be less than or equal to VALUES',
        }

        self.validation = validation

    def get_validation_type(self, function_base_name):
        if function_base_name in self.sentence_conventions:
            return self.sentence_conventions[function_base_name]
        else:
            return 'Custom validator: ' + str(function_base_name)

    def __str__(self):
        arguments = self.validation.arguments
        validation = self.get_validation_type(self.validation.function_base_name)
        if validation.__contains__('VALUES'):
            validation = validation.replace('VALUES', str(arguments[0]))
        elif arguments:
            validation += ': ' + str(arguments[0])

        return ' - ' + validation


class ParameterDetailMarkdown:
    @typechecked
    def __init__(self, declare_parameters: DeclareParameter):
        self.declare_parameters = declare_parameters
        self.param_validations = [
            ParameterValidationMarkdown(val)
            for val in declare_parameters.parameter_validations
        ]

    def __str__(self):
        constraints = '\n'.join(str(val) for val in self.param_validations)

        data = {
            'name': self.declare_parameters.parameter_name,
            'read_only': self.declare_parameters.parameter_read_only,
            'type': self.declare_parameters.code_gen_variable.defined_type,
            'default_value': self.declare_parameters.code_gen_variable.lang_str_value,
            'constraints': constraints,
            'additional_constraints': self.declare_parameters.parameter_additional_constraints,
            # remove leading whitespace from description, this is necessary for correct indentation of multi-line descriptions
            'description': re.sub(
                r'(?m)^(?!$)\s*',
                '',
                str(self.declare_parameters.parameter_description),
                flags=re.MULTILINE,
            ),
        }

        j2_template = Template(GenerateCode.templates['parameter_detail'])
        code = j2_template.render(data, trim_blocks=True)
        return code


class RuntimeParameterDetailMarkdown:
    @typechecked
    def __init__(self, declare_parameters: DeclareRuntimeParameter):
        self.declare_parameters = declare_parameters
        self.param_validations = [
            ParameterValidationMarkdown(val)
            for val in declare_parameters.parameter_validations
        ]

    def __str__(self):
        constraints = '\n'.join(str(val) for val in self.param_validations)
        data = {
            # replace __map_key with <key>
            'name': re.sub(
                r'__map_(\w+)',
                lambda match: '<' + match.group(1) + '>',
                self.declare_parameters.parameter_name,
            ),
            'read_only': self.declare_parameters.parameter_read_only,
            'type': self.declare_parameters.code_gen_variable.defined_type,
            'default_value': self.declare_parameters.code_gen_variable.lang_str_value,
            'constraints': constraints,
            'additional_constraints': self.declare_parameters.parameter_additional_constraints,
            'description': self.declare_parameters.parameter_description,
        }

        j2_template = Template(GenerateCode.templates['parameter_detail'])
        code = j2_template.render(data, trim_blocks=True)
        return code


class DefaultConfigMarkdown:
    @typechecked
    def __init__(self, gen_param_struct: GenerateCode):
        self.gen_param_struct = gen_param_struct
        self.param_details = [
            ParameterDetailMarkdown(param)
            for param in self.gen_param_struct.declare_parameters
        ]

    def __str__(self):
        j2_template = Template(GenerateCode.templates['default_config'])

        tmp = (
            '\n'.join(
                param.parameter_name
                + ': '
                + str(param.code_gen_variable.lang_str_value)
                for param in self.gen_param_struct.declare_parameters
            )
            + '\n'
            + '\n'.join(
                # replace __map_key with <key>
                re.sub(
                    r'__map_(\w+)',
                    lambda match: '<' + match.group(1) + '>',
                    param.parameter_name,
                )
                + ': '
                + str(param.code_gen_variable.lang_str_value)
                for param in self.gen_param_struct.declare_dynamic_parameters
            )
        )

        # Split the string into lines and group them by the first part
        def nest_dict(d, keys, value):
            # Check if the value is a string
            if isinstance(value, str):
                # Remove double quotes from the string
                value = value.replace('"', '')
                # Try to convert the value to a boolean, number, or leave it as a string
                if value.lower() == 'true':
                    value = True
                elif value.lower() == 'false':
                    value = False
                else:
                    try:
                        value = float(value)
                    except ValueError:
                        pass

            if len(keys) == 1:
                d[keys[0]] = value
            else:
                key = keys.pop(0)
                if key not in d:
                    d[key] = {}
                nest_dict(d[key], keys, value)

        # Split the string into lines and create a dictionary
        d = {}
        for line in tmp.strip().split('\n'):
            name, value = line.split(':', 1)
            keys = name.split('.')
            nest_dict(d, keys, value.strip())

        # Convert the dictionary to a string
        result = yaml.dump(d, default_flow_style=False)

        data = {
            'namespace': self.gen_param_struct.namespace,
            'default_param_values': result,
        }
        code = j2_template.render(data, trim_blocks=True)

        return code


class AutoDocumentation:
    @typechecked
    def __init__(self, gen_param_struct: GenerateCode):
        self.gen_param_struct = gen_param_struct
        self.default_config = DefaultConfigMarkdown(gen_param_struct)
        self.param_details = [
            ParameterDetailMarkdown(param)
            for param in self.gen_param_struct.declare_parameters
        ]
        self.runtime_param_details = [
            RuntimeParameterDetailMarkdown(param)
            for param in self.gen_param_struct.declare_dynamic_parameters
        ]

    def __str__(self):
        words = self.gen_param_struct.namespace.split('_')
        title = ' '.join(word.capitalize() for word in words)

        data = {
            'title': title,
            'default_config': str(self.default_config),
            'parameter_details': '\n'.join(str(val) for val in self.param_details)
            + '\n'
            + '\n'.join(str(val) for val in self.runtime_param_details),
        }

        j2_template = Template(GenerateCode.templates['documentation'])
        code = j2_template.render(data, trim_blocks=True)

        return code


def run(yaml_file, output_file, language):
    # cpp is used here because it the desired style of the markdown, e.g. false for C++ instead of False for Python
    gen_param_struct = GenerateCode(language)
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    gen_param_struct.parse(yaml_file, '')

    auto_doc = AutoDocumentation(gen_param_struct)

    docs = str(auto_doc)
    with open(output_file, 'w') as f:
        f.write(docs)
    pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output_markdown_file')
    parser.add_argument('--input_yaml_file')
    parser.add_argument('--language', default='markdown')
    args = parser.parse_args()
    run(args.input_yaml_file, args.output_markdown_file, args.language)
    print(args)


if __name__ == '__main__':
    sys.exit(main())
