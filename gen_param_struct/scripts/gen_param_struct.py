#!/usr/bin/env python3

import yaml
from yaml.parser import ParserError
import sys
import os


# class to help minimize string copies
class Buffer:
    def __init__(self):
        self.data_ = bytearray()

    def __iadd__(self, element):
        self.data_.extend(element.encode())
        return self

    def __str__(self):
        return self.data_.decode()


class YAMLSyntaxError(Exception):
    """Raised when the input value is too large"""

    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


def compile_error(msg):
    # sys.stderr.write("ERROR: " + msg)
    return YAMLSyntaxError("\nERROR: " + msg)


def array_type(defined_type):
    return defined_type.__contains__("array")


def validate_type(defined_type, value):
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

def bool_to_str(cond):
    return "true" if cond else "false"


def float_to_str(num):
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


def int_to_str(num):
    return str(num)


def str_to_str(s):
    return "\"%s\"" % s


def get_translation_data(defined_type):
    if defined_type == 'string_array':
        cpp_type = 'std::string'
        parameter_conversion = 'as_string_array()'
        val_to_cpp_str = str_to_str

    elif defined_type == 'double_array':
        cpp_type = 'double'
        parameter_conversion = 'as_double_array()'
        val_to_cpp_str = float_to_str

    elif defined_type == 'int_array':
        cpp_type = 'int'
        parameter_conversion = 'as_integer_array()'
        val_to_cpp_str = int_to_str

    elif defined_type == 'bool_array':
        cpp_type = 'bool'
        parameter_conversion = 'as_bool_array()'
        val_to_cpp_str = bool_to_str

    elif defined_type == 'string':
        cpp_type = 'std::string'
        parameter_conversion = 'as_string()'
        val_to_cpp_str = str_to_str

    elif defined_type == 'double':
        cpp_type = 'double'
        parameter_conversion = 'as_double()'
        val_to_cpp_str = float_to_str

    elif defined_type == 'integer':
        cpp_type = 'int'
        parameter_conversion = 'as_int()'
        val_to_cpp_str = int_to_str

    elif defined_type == 'bool':
        cpp_type = 'bool'
        parameter_conversion = 'as_bool()'
        val_to_cpp_str = bool_to_str

    else:
        raise compile_error('invalid yaml type: %s' % type(defined_type))

    return cpp_type, val_to_cpp_str, parameter_conversion


def declare_struct(defined_type, cpp_type, val_to_cpp_str, name, default_value):
    code_str = Buffer()
    if array_type(defined_type):
        code_str += "std::vector<%s> %s_ " % (cpp_type, name)
        if default_value:
            code_str += "= {"
            for ind, val in enumerate(default_value[:-1]):
                code_str += "%s, " % val_to_cpp_str(val)
            code_str += "};\n"
        else:
            code_str += ";\n"
    else:
        if default_value:
            code_str += "%s %s_ = %s;\n" % (cpp_type, name, val_to_cpp_str(default_value))
        else:
            code_str += "%s %s_;\n" % (cpp_type, name)
    return str(code_str)


def if_statement(effects, conditions, bool_operators):
    code_str = Buffer()
    code_str += "if ("
    code_str += conditions[0]
    for ind, condition in enumerate(conditions[1:]):
        code_str += bool_operators[ind]
        code_str += condition
    code_str += ") {\n"
    for effect in effects:
        code_str += effect
    code_str += "}\n"
    return str(code_str)


def scoped_codeblock(effects):
    code_str = Buffer()
    code_str += "{\n"
    for effect in effects:
        code_str += effect
    code_str += "}\n"
    return str(code_str)


# class used to fill template text file with passed in yaml file
class GenParamStruct:

    def __init__(self):
        self.contents = ""
        self.struct = Buffer()
        self.param_set = Buffer()
        self.param_describe = Buffer()
        self.param_get = Buffer()
        self.target = ""

    def parse_params(self, name, value, nested_name_list):
        # define names for parameters and variables
        nested_name = "".join(x + "_." for x in nested_name_list[1:])
        param_prefix = "p_" + "".join(x + "_" for x in nested_name_list[1:])
        # param_prefix += "".join(x + "_" for x in nested_name_list[1:])
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        # optional attributes
        default_value = value.get('default_value')
        description = value.get('description')
        read_only = value.get('read_only')
        bounds = value.get('bounds')
        fixed_size = value.get('fixed_size')

        # required attributes
        try:
            defined_type = value['type']
        except KeyError as e:
            raise compile_error("No type defined for parameter %s" % param_name)

        # validate inputs
        if bounds and not validate_type(defined_type, bounds):
            raise compile_error("The type of the bounds must be the same type as the defined type")
        if default_value and not validate_type(defined_type, default_value):
            raise compile_error("The type of the default_value must be the same type as the defined type")
        if fixed_size and not isinstance(fixed_size, int):
            raise compile_error("The type of the fixed size attribute must be an integer")

        # get translation variables from defined value type
        cpp_type, val_to_cpp_str, parameter_conversion = get_translation_data(defined_type)

        self.struct += declare_struct(defined_type, cpp_type, val_to_cpp_str, name, default_value)

        # set param value if param.name if param_name
        param_set_effect = ["params_.%s_ = param.%s;\n" % (nested_name + name, parameter_conversion)]
        param_set_conditions = ["param.get_name() == " + "\"%s\" " % param_name]
        param_set_bool_operators = []
        if bounds:
            param_set_conditions.append("validate_bounds(\"%s\", param.%s, %s, %s, result) " % (
                param_name, parameter_conversion, val_to_cpp_str(bounds[0]), val_to_cpp_str(bounds[1])))
            param_set_bool_operators.append("&&")
        if fixed_size:
            param_set_conditions.append("validate_length(\"%s\", param.%s, %s, result) " % (
                param_name, parameter_conversion, int_to_str(fixed_size)))
            param_set_bool_operators.append("&&")

        self.param_set += if_statement(param_set_effect, param_set_conditions, param_set_bool_operators)

        # create parameter description and add to map
        param_describe_effects = ["rcl_interfaces::msg::ParameterDescriptor descriptor;\n",
                                  "descriptor.description = \"%s\";\n" % description]
        if bounds:
            param_describe_effects.extend([
                "rcl_interfaces::msg::FloatingPointRange range;\n",
                "range.from_value = %s;\n" % val_to_cpp_str(bounds[0]),
                "range.to_value = %s;\n" % val_to_cpp_str(bounds[1]),
                "descriptor.floating_point_range.push_back(range);\n"
            ])
        param_describe_effects.extend([
            "descriptor.read_only = %s;\n" % bool_to_str(read_only),
            "desc_map[\"%s\"] = descriptor;\n" % param_name
        ])
        if default_value:
            param_describe_effects.extend([
                "if (!parameters_interface->has_parameter(\"%s\")){\n" % param_name,
                "auto %s = rclcpp::ParameterValue(params_.%s_);\n" % (
                    param_prefix + name, nested_name + name),
                "parameters_interface->declare_parameter(\"%s\", %s, descriptor);\n" % (
                    param_name, param_prefix + name),
                "}\n"
            ])
        self.param_describe += scoped_codeblock(param_describe_effects)

        # get parameter from by calling parameters_interface API
        param_get_effect = ["throw rclcpp::exceptions::InvalidParameterValueException(\"Invalid value set during "
                            "initialization for parameter %s \");" % param_name]

        param_get_conditions = []
        param_get_bool_operators = []
        if fixed_size:
            param_get_conditions.append("!validate_length(parameters_interface->get_parameter(\"%s\").%s, %s) " % (
                param_name, parameter_conversion, int_to_str(fixed_size)))
            param_get_bool_operators.append("||")
        if bounds:
            param_get_conditions.append("!validate_bounds(parameters_interface->get_parameter(\"%s\").%s, %s, %s) " % (
                param_name, parameter_conversion, val_to_cpp_str(bounds[0]), val_to_cpp_str(bounds[1])))
            param_get_bool_operators.append("||")
        if len(param_get_conditions) > 0:
            self.param_get += if_statement(param_get_effect, param_get_conditions, param_get_bool_operators)
        self.param_get += "params_.%s_ = parameters_interface->get_parameter(\"%s\").%s;\n" % (
            nested_name + name, param_name, parameter_conversion)

    def parse_dict(self, name, root_map, nested_name):
        if isinstance(root_map, dict) and isinstance(next(iter(root_map.values())), dict):
            if name != self.target:
                self.struct += "struct %s {\n" % name
            for key in root_map:
                if isinstance(root_map[key], dict):
                    nested_name.append(name)
                    self.parse_dict(key, root_map[key], nested_name)
                    nested_name.pop()
            if name != self.target:
                self.struct += "} %s_;\n" % name
        else:
            self.parse_params(name, root_map, nested_name)

    def run(self):
        if len(sys.argv) != 3:
            raise compile_error("generate_param_struct_header expects three input argument: target, output directory, "
                                "and yaml file path")

        param_gen_directory = sys.argv[0].split("/")
        param_gen_directory = "".join(x + "/" for x in param_gen_directory[:-1])

        out_directory = sys.argv[1]
        if out_directory[-1] != "/":
            out_directory += "/"
        if param_gen_directory[-1] != "/":
            param_gen_directory += "/"

        if not os.path.isdir(out_directory):
            raise compile_error("The specified output directory: %s does not exist" % out_directory)

        yaml_file = sys.argv[2]
        with open(yaml_file) as f:
            try:
                docs = yaml.load_all(f, Loader=yaml.FullLoader)
                doc = list(docs)[0]
            except ParserError as e:
                raise compile_error(str(e))

            if len(doc) != 1:
                raise compile_error("the controller yaml definition must only have one root element")
            self.target = list(doc.keys())[0]
            self.parse_dict(self.target, doc[self.target], [])

        COMMENTS = "// this is auto-generated code "
        INCLUDES = "#include <rclcpp/node.hpp>\n#include <vector>\n#include <string>"
        NAMESPACE = self.target + "_parameters"

        with open(param_gen_directory + "/templates/template.txt", "r") as f:
            self.contents = f.read()

        self.contents = self.contents.replace("**COMMENTS**", COMMENTS)
        self.contents = self.contents.replace("**INCLUDES**", INCLUDES)
        self.contents = self.contents.replace("**NAMESPACE**", NAMESPACE)
        self.contents = self.contents.replace("**STRUCT_NAME**", str(self.target))
        self.contents = self.contents.replace("**STRUCT_CONTENT**", str(self.struct))
        self.contents = self.contents.replace("**PARAM_SET**", str(self.param_set))
        self.contents = self.contents.replace("**DESCRIBE_PARAMS**", str(self.param_describe))
        self.contents = self.contents.replace("**GET_PARAMS**", str(self.param_get))

        with open(out_directory + self.target + ".h", "w") as f:
            f.write(self.contents)


if __name__ == "__main__":
    gen_param_struct = GenParamStruct()
    gen_param_struct.run()
