#!/usr/bin/env python3

import yaml
from yaml.parser import ParserError
import sys
from code_gen_utils import *


def get_all_templates():
    template_path = os.path.join(os.path.dirname(__file__), 'jinja_templates')
    template_map = dict()
    for file_name in [f for f in os.listdir(template_path) if os.path.isfile(os.path.join(template_path, f))]:
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
            defined_type = value['type']
        except KeyError as e:
            raise compile_error("No type defined for parameter %s" % param_name)

        # optional attributes
        default_value = value.get('default_value', None)
        description = value.get('description', '')
        read_only = bool(value.get('read_only', False))
        bounds = value.get('bounds', None)
        fixed_size = value.get('fixed_size', None)
        validations = value.get('validation', [])

        # # validate inputs
        if bounds is not None and not validate_type(defined_type, bounds):
            raise compile_error("The type of the bounds must be the same type as the defined type")
        if default_value and not validate_type(defined_type, default_value):
            raise compile_error("The type of the default_value must be the same type as the defined type")
        if fixed_size is not None and not isinstance(fixed_size, int):
            raise compile_error("The type of the fixed size attribute must be an integer")

        # add default validations if applicable
        if len(validations) > 0 and isinstance(validations[0], str):
            validations = [validations]

        if bounds is not None:
            validations.append(['validate_' + defined_type + '_bounds', bounds[0], bounds[1]])

        if fixed_size is not None:
            validations.append(['validate_' + defined_type + '_len', fixed_size])

        # define struct
        var = VariableDeclaration(defined_type, name, default_value)
        self.struct_tree.add_field(var)

        # declare parameter
        declare_parameter = DeclareParameter(param_name, description, read_only, defined_type, default_value)
        self.declare_parameters.append(declare_parameter)

        # update parameter
        update_parameter_invalid = update_parameter_fail_validation()
        update_parameter_valid = update_parameter_pass_validation()
        parameter_conversion = get_parameter_as_function_str(defined_type)
        update_parameter = UpdateParameter(param_name, parameter_conversion)
        for validation in validations:
            validation_function = ValidationFunction(validation[0], validation[1:])
            parameter_validation = ParameterValidation(update_parameter_invalid, update_parameter_valid,
                                                       validation_function)
            update_parameter.add_parameter_validation(parameter_validation)

        self.update_parameters.append(update_parameter)

        # set parameter
        declare_parameter_invalid = initialization_fail_validation(param_name)
        declare_parameter_valid = initialization_pass_validation(param_name, parameter_conversion)
        declare_parameter_set = DeclareParameterSet(param_name, parameter_conversion)
        for validation in validations:
            validation_function = ValidationFunction(validation[0], validation[1:])
            parameter_validation = ParameterValidation(declare_parameter_invalid, declare_parameter_valid,
                                                       validation_function)
            declare_parameter_set.add_parameter_validation(parameter_validation)

        self.declare_parameter_sets.append(declare_parameter_set)

    def parse_dict(self, name, root_map, nested_name):

        if isinstance(root_map, dict) and isinstance(next(iter(root_map.values())), dict):
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
        data = {'USER_VALIDATORS': self.user_validation,
                'COMMENTS': self.comments,
                'namespace': self.namespace,
                'validation_functions': self.validation_functions,
                'struct_content': self.struct_tree.inner_content(),
                'update_params_set': "\n".join([str(x) for x in self.update_parameters]),
                'declare_params': "\n".join([str(x) for x in self.declare_parameters]),
                'declare_params_set': "\n".join([str(x) for x in self.declare_parameter_sets])}

        j2_template = Template(GenParamStruct.templates['parameter_listener'])
        code = j2_template.render(data, trim_blocks=True)
        return code

    def run(self):
        if 3 > len(sys.argv) > 4:
            raise compile_error("generate_parameter_library_py expects three input argument: output_file, "
                                "yaml file path, [validate include header]")

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
                raise compile_error("the controller yaml definition must only have one root element")
            self.namespace = list(doc.keys())[0]
            self.parse_dict(self.namespace, doc[self.namespace], [])

        if len(sys.argv) > 3:
            user_validation_file = sys.argv[3]
            with open(user_validation_file, 'r') as f:
                self.user_validation = f.read()

        validation_functions_file = os.path.join(
            os.path.dirname(__file__), 'cpp_templates', 'validators.hpp')
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
