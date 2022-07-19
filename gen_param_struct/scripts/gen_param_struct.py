#!/usr/bin/env python3

import yaml
from yaml.parser import ParserError
import sys
import os
from typing import Callable
from typeguard import typechecked

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
def bool_to_str(cond: bool):
    return "true" if cond else "false"


@typechecked
def float_to_str(num: float):
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
def int_to_str(num: int):
    return str(num)


@typechecked
def str_to_str(s: str):
    return "\"%s\"" % s


# cpp_type, val_to_cpp_str, parameter_conversion
@typechecked
def get_translation_data(defined_type: str) -> (str, Callable, str):
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


@typechecked
def get_validation_translation(validation_functions: list[list[any]]):
    if not isinstance(validation_functions[0], list):
        validation_functions = [validation_functions]
    for validation_function in validation_functions:
        if len(validation_function) < 2:
            raise compile_error(
                "The user yaml defined validation function %s does not have enough input arguments, requires at least 1." %
                validation_function[0])
        for ind, arg in enumerate(validation_function[1:]):
            if isinstance(arg, list):
                raise compile_error(
                    "The user yaml defined validation function %s uses a list input argument which is not supported" %
                    validation_function[0])

            if isinstance(arg, int):
                val_to_cpp_str = int_to_str
            elif isinstance(arg, float):
                val_to_cpp_str = float_to_str
            elif isinstance(arg, bool):
                val_to_cpp_str = bool_to_str
            elif isinstance(arg, str):
                val_to_cpp_str = str_to_str
            else:
                raise compile_error('invalid python type pass to get_validation_translation, type: %s' % type(arg))

            validation_function[ind + 1] = val_to_cpp_str(arg)

    return validation_functions


@typechecked
def declare_struct(defined_type: str, cpp_type: str, name: str, default_value: list) -> str:
    code_str = Buffer()
    if array_type(defined_type):
        code_str += "std::vector<%s> %s_ " % (cpp_type, name)
        if len(default_value) > 0:
            code_str += "= {"
            for ind, val in enumerate(default_value[:-1]):
                code_str += "%s, " % val
            code_str += "};\n"
        else:
            code_str += ";\n"
    else:
        if len(default_value) > 0:
            code_str += "%s %s_ = %s;\n" % (cpp_type, name, default_value[0])
        else:
            code_str += "%s %s_;\n" % (cpp_type, name)
    return str(code_str)


@typechecked
def if_else_statement(effects_true, effects_false, conditions, bool_operators) -> str:
    code_str = Buffer()
    code_str += "if ("
    code_str += conditions[0]
    for ind, condition in enumerate(conditions[1:]):
        code_str += bool_operators[ind]
        code_str += condition
    code_str += ") {\n"
    for effect in effects_true:
        code_str += effect
    if len(effects_false) > 0:
        code_str += "} else {"
        for effect in effects_false:
            code_str += effect
    code_str += "}\n"

    return str(code_str)


@typechecked
def if_statement(effects, conditions, bool_operators) -> str:
    return if_else_statement(effects, [], conditions, bool_operators)


@typechecked
def flatten_effects(effects) -> str:
    code_str = Buffer()
    for effect in effects:
        code_str += effect

    return str(code_str)


@typechecked
def scoped_codeblock(effects) -> str:
    code_str = Buffer()
    code_str += "{\n"
    code_str += flatten_effects(effects)
    code_str += "}\n"
    return str(code_str)


@typechecked
def function_call(namespace: str, func_name: str, args: list[str]) -> str:
    code_str = Buffer()
    code_str += namespace + "::" + func_name
    code_str += "("
    for ind, arg in enumerate(args):
        code_str += arg
        if ind < len(args) - 1:
            code_str += ", "
    code_str += ")"

    return str(code_str)


@typechecked
def validation_sequence(namespace: str, func_name: str, args: list, effects_true: list[str], effects_false: list[str]):
    # assumes that the validation function is named validate_{defined_type}_{method}
    if len(args) == 0:
        return flatten_effects(effects_true)  # no validation needed
    tmp = ["param"]
    tmp.extend(args)
    args = tmp
    code_str = Buffer()
    code_str += "validation_result = "
    code_str += function_call(namespace, func_name, args)
    code_str += ";\n"

    conditions = ["validation_result.success()"]
    code_str += if_else_statement(effects_true, effects_false, conditions, [])

    return str(code_str)


@typechecked
def default_validation(effects: list[str], defined_type: str, fixed_size: list, bounds: list) -> str:
    code_str = Buffer()
    effects_false = ["result.reason = validation_result.error_msg();\n",
                     "result.successful = false;"]
    if fixed_size:
        effects2 = [
            validation_sequence("gen_param_struct_validators", "validate_" + defined_type + "_len", fixed_size,
                                effects,
                                effects_false)]
    else:
        effects2 = effects
    if bounds:
        code_str += validation_sequence("gen_param_struct_validators", "validate_" + defined_type + "_bounds", bounds,
                                        effects2,
                                        effects_false)
    else:
        code_str += flatten_effects(effects2)

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
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        # optional attributes
        default_value = value.get('default_value')
        description = value.get('description')
        read_only = value.get('read_only')
        bounds = value.get('bounds')
        fixed_size = value.get('fixed_size')
        validation = value.get('validation')

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

        # convert python types to lists of inputs
        if default_value:
            if array_type(defined_type):
                for i in range(len(default_value)):
                    default_value[i] = val_to_cpp_str(default_value[i])
            else:
                default_value = [val_to_cpp_str(default_value)]
        else:
            default_value = []
        if bounds:
            for i in range(len(bounds)):
                bounds[i] = val_to_cpp_str(bounds[i])
        else:
            bounds = []
        if fixed_size:
            fixed_size = [int_to_str(fixed_size)]
        else:
            fixed_size = []
        if read_only:
            read_only = [read_only]
        else:
            read_only = []
        if validation:
            if not isinstance(validation[0], list):
                validation = [validation]
            validation = get_validation_translation(validation)
        else:
            validation = []

        # define struct
        self.struct += declare_struct(defined_type, cpp_type, name, default_value)

        # set param value if param.name is the parameter being updated
        param_set_effects = ["params_.%s_ = param.%s;\n" % (nested_name + name, parameter_conversion),
                             "result.successful = true;\n"]
        param_set_conditions = ["param.get_name() == " + "\"%s\" " % param_name]
        code_str = default_validation(param_set_effects, defined_type, fixed_size, bounds)
        self.param_set += if_statement(code_str, param_set_conditions, [])

        # create parameter description
        param_describe_effects = ["rcl_interfaces::msg::ParameterDescriptor descriptor;\n",
                                  "descriptor.description = \"%s\";\n" % description]
        if len(bounds):
            param_describe_effects.extend([
                "rcl_interfaces::msg::FloatingPointRange range;\n",
                "range.from_value = %s;\n" % bounds[0],
                "range.to_value = %s;\n" % bounds[1],
                "descriptor.floating_point_range.push_back(range);\n"
            ])
        if len(read_only):
            param_describe_effects.extend([
                "descriptor.read_only = %s;\n" % bool_to_str(read_only[0]),
            ])
        if len(default_value):
            value_str = "rclcpp::ParameterValue(params_.%s_)" % (nested_name + name)
        else:
            value_str = "rclcpp::ParameterType::PARAMETER_%s" % defined_type.upper()

        param_describe_effects_2 = ["auto %s = %s;\n" % (param_prefix + name, value_str),
                                    "parameters_interface->declare_parameter(\"%s\", %s, descriptor);\n" % (
                                        param_name, param_prefix + name)]
        param_describe_conditions = ["!parameters_interface->has_parameter(\"%s\")" % param_name]

        param_describe_effects.append(
            if_statement(param_describe_effects_2, param_describe_conditions, [])
        )
        self.param_describe += scoped_codeblock(param_describe_effects)

        # get parameter from by calling parameters_interface API
        self.param_get += "param = parameters_interface->get_parameter(\"%s\");\n" % param_name
        param_get_effect_false = ["throw rclcpp::exceptions::InvalidParameterValueException(\"Invalid value set during "
                                  "initialization for parameter %s: \" + validation_result.error_msg());" % param_name]
        param_get_effect_true = ["params_.%s_ = param.%s;\n" % (
            nested_name + name, parameter_conversion)]

        # add default validation
        code_str = validation_sequence("gen_param_struct_validators", "validate_" + defined_type + "_len", fixed_size,
                                       param_get_effect_true,
                                       param_get_effect_false)
        code_str = validation_sequence("gen_param_struct_validators", "validate_" + defined_type + "_bounds", bounds,
                                      [code_str],
                                      param_get_effect_false)
        # add custom validation
        for val in validation:
            code_str = validation_sequence("gen_param_struct_validators", val[0], val[1:], [code_str],
                                          param_get_effect_false)

        self.param_get += code_str

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
        INCLUDES = "#include <gen_param_struct/validators.hpp>"
        NAMESPACE = self.target + "_parameters"

        with open(param_gen_directory + "/templates/template.txt", "r") as f:
            self.contents = f.read()

        self.contents = self.contents.replace("**COMMENTS**", COMMENTS)
        self.contents = self.contents.replace("**INCLUDES**", INCLUDES)
        self.contents = self.contents.replace("**NAMESPACE**", NAMESPACE)
        self.contents = self.contents.replace("**CLASS_NAME**", str(self.target))
        self.contents = self.contents.replace("**STRUCT_CONTENT**", str(self.struct))
        self.contents = self.contents.replace("**PARAM_SET**", str(self.param_set))
        self.contents = self.contents.replace("**DESCRIBE_PARAMS**", str(self.param_describe))
        self.contents = self.contents.replace("**GET_PARAMS**", str(self.param_get))

        with open(out_directory + self.target + ".h", "w") as f:
            f.write(self.contents)


if __name__ == "__main__":
    gen_param_struct = GenParamStruct()
    gen_param_struct.run()
