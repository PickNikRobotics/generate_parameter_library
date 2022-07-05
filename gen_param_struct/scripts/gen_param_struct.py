#!/usr/bin/env python3

import yaml
import sys
import os


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


# class to help minimize string copies
class Buffer:
    def __init__(self):
        self.data_ = bytearray()

    def __iadd__(self, element):
        self.data_.extend(element.encode())
        return self

    def __str__(self):
        return self.data_.decode()


# class used to fill template.txt using passed in yaml file
class GenParamStruct:

    def __init__(self):
        self.contents = ""
        self.struct = Buffer()
        self.param_set = Buffer()
        self.param_describe = Buffer()
        self.param_get = Buffer()
        self.target = ""

    def parse_params(self, name, value, nested_name_list):

        nested_name = "".join(x + "_." for x in nested_name_list[1:])

        default_value = value['default_value']
        description = value['description']
        configurable = value['configurable']
        optional = value["optional"]
        bounds = []
        if "bounds" in value:
            bounds = value["bounds"]

        if isinstance(default_value, list):
            default_value_type = type(default_value[0])
        else:
            default_value_type = type(default_value)

        if isinstance(default_value, list):
            if default_value_type is str:
                data_type = "std::string"
                conversion_func = "as_string_array()"
                str_fun = str_to_str

            elif default_value_type is float:
                data_type = "double"
                conversion_func = "as_double_array()"
                str_fun = float_to_str

            elif default_value_type is int and default_value_type is not bool:
                data_type = "int"
                conversion_func = "as_integer_array()"
                str_fun = int_to_str

            elif default_value_type is bool:
                data_type = "bool"
                conversion_func = "as_bool_array()"
                str_fun = bool_to_str

            else:
                sys.stderr.write("invalid yaml type: %s" % type(default_value[0]))
                raise AssertionError()

            self.struct += "std::vector<%s> %s_ = {" % (data_type, name)
            for ind, val in enumerate(default_value[:-1]):
                self.struct += "%s, " % str_fun(val)
            self.struct += "%s};\n" % str_fun(default_value[-1])

        else:
            if default_value_type is str:
                data_type = "std::string"
                conversion_func = "as_string()"
                str_fun = str_to_str

            elif default_value_type is float:
                data_type = "double"
                conversion_func = "as_double()"
                str_fun = float_to_str

            elif default_value_type is int and default_value_type is not bool:
                data_type = "int"
                conversion_func = "as_int()"
                str_fun = int_to_str

            elif default_value_type is bool:
                data_type = "bool"
                conversion_func = "as_bool()"

                def str_fun(cond):
                    return bool_to_str(cond)
            else:
                sys.stderr.write("invalid yaml type: %s" % type(default_value))
                raise AssertionError()

            self.struct += "%s %s_ = %s;\n" % (data_type, name, str_fun(default_value))

        param_prefix = "p_"
        param_prefix += "".join(x + "_" for x in nested_name_list[1:])
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        self.param_set += "if (param.get_name() == " + "\"%s\" " % param_name
        if isinstance(default_value, list):
            self.param_set += "&& validate_length(\"%s\", param.%s, %s, result) " % (param_name, conversion_func, int_to_str(len(default_value)))
        if len(bounds) > 0:
            self.param_set += "&& validate_bounds(\"%s\", param.%s, %s, %s, result) " % (param_name, conversion_func, str_fun(bounds[0]), str_fun(bounds[1]) )
        self.param_set += ") {\n"

        self.param_set += "params_.%s_ = param.%s;\n" % (nested_name + name, conversion_func)
        self.param_set += "}\n"


        self.param_describe += "{\n"
        self.param_describe += "rcl_interfaces::msg::ParameterDescriptor descriptor;\n"
        self.param_describe += "descriptor.description = \"%s\";\n" % description
        if len(bounds) > 0:
            if default_value_type is not type(bounds[0]):
                sys.stderr.write("The type of the bounds must be the same as the default value")
                raise AssertionError()
            self.param_describe += "rcl_interfaces::msg::FloatingPointRange range;\n"
            self.param_describe += "range.from_value = %s;\n" % str_fun(bounds[0])
            self.param_describe += "range.to_value = %s;\n" % str_fun(bounds[1])
            self.param_describe += "descriptor.floating_point_range.push_back(range);\n"
        self.param_describe += "descriptor.read_only = %s;\n" % bool_to_str(not configurable)
        self.param_describe += "desc_map[\"%s\"] = descriptor;\n" % param_name

        if optional:
            self.param_describe += "if (!parameters_interface->has_parameter(\"%s\")){\n" % param_name
            self.param_describe += "auto %s = rclcpp::ParameterValue(params_.%s_);\n" % (
                param_prefix + name, nested_name + name)
            self.param_describe += "parameters_interface->declare_parameter(\"%s\", %s, descriptor);\n" % (
                param_name, param_prefix + name)
            self.param_describe += "}\n"

        self.param_describe += "}\n"

        # self.param_describe += "parameters_interface->declare_parameter(\"%s\", %s, descriptor);\n" % (
        #     param_name, param_prefix + name)
        # self.param_describe += "}"

        if (isinstance(default_value, list) and default_value[0] != "UNDEFINED") or len(bounds) > 0:
            self.param_get += "if ("
            logical = ""
            if isinstance(default_value, list) and default_value[0] != "UNDEFINED":
                self.param_get += "!validate_length(parameters_interface->get_parameter(\"%s\").%s, %s) " % (param_name, conversion_func, int_to_str(len(default_value)))
                logical = " || "
            if len(bounds) > 0:
                self.param_get += "%s !validate_bounds(parameters_interface->get_parameter(\"%s\").%s, %s, %s) " % (logical, param_name, conversion_func, str_fun(bounds[0]), str_fun(bounds[1]) )
            self.param_get += ") {\n"
            self.param_get += "throw rclcpp::exceptions::InvalidParameterValueException(\"Invalid value set during initialization for parameter %s \");" % param_name
            self.param_get += "}\n"

        self.param_get += "params_.%s_ = parameters_interface->get_parameter(\"%s\").%s;" % (
            nested_name + name, param_name, conversion_func)


        self.param_get += "\n"

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
            sys.stderr.write("generate_param_struct_header expects four input argument: target, output directory, "
                             " and yaml file path")
            raise AssertionError()

        param_gen_directory = sys.argv[0].split("/")
        param_gen_directory = "".join(x + "/" for x in param_gen_directory[:-1])

        out_directory = sys.argv[1]
        if out_directory[-1] != "/":
            out_directory += "/"
        if param_gen_directory[-1] != "/":
            param_gen_directory += "/"

        if not os.path.isdir(out_directory):
            sys.stderr.write("The specified output directory: %s does not exist" % out_directory)
            raise AssertionError()

        yaml_file = sys.argv[2]
        with open(yaml_file) as f:
            docs = yaml.load_all(f, Loader=yaml.FullLoader)

            doc = list(docs)[0]
            if len(doc) != 1:
                sys.stderr.write("the controller yaml definition must only have one root element")
                raise AssertionError()
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
