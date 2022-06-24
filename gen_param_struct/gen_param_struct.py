#!/usr/bin/env python3

import yaml
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


# class used to fill template.txt using passed in yaml file
class GenParamStruct:

    def __init__(self):
        self.contents = ""
        self.struct = Buffer()
        self.param_set = Buffer()
        self.param_declare = Buffer()
        self.target = ""

    def parse_params(self, name, value, nested_name_list):

        nested_name = "".join(x + "_." for x in nested_name_list[1:])

        if isinstance(value, list) and isinstance(value[0], str):
            self.struct += "std::vector<std::string> %s = {" % name
            for ind, val in enumerate(value[:-1]):
                self.struct += "\"%s\", " % val
            self.struct += "\"%s\"};\n" % value[-1]

            self.param_set += "if (param.get_name() == " + "\"%s\") {\n" % name
            self.param_set += "%s = param.as_string_array();\n" % (nested_name + name)
            self.param_set += "}\n"

        elif isinstance(value, list) and isinstance(value[0], float):
            self.struct += "std::vector<double> %s = {" % name
            for ind, val in enumerate(value[:-1]):
                self.struct += "\"%s\", " % val
            self.struct += "\"%s\"};\n" % value[-1]

            self.param_set += "if (param.get_name() == " + "\"%s\") {\n" % name
            self.param_set += "%s = param.as_double_array();\n" % (nested_name + name)
            self.param_set += "}\n"

        elif isinstance(value, str):
            self.struct += "std::string %s = \"%s\";\n" % (name, value)

            self.param_set += "if (param.get_name() == " + "\"%s\") {\n" % name
            self.param_set += "%s = param.as_string();\n" % (nested_name + name)
            self.param_set += "}\n"

        elif isinstance(value, float):
            self.struct += "double %s = %s;\n" % (name, str(value))

            self.param_set += "if (param.get_name() == " + "\"%s\") {\n" % name
            self.param_set += "%s = param.as_double();\n" % (nested_name + name)
            self.param_set += "}\n"

        elif isinstance(value, bool):
            if value:
                tmp = "true"
            else:
                tmp = "false"
            self.struct += "bool %s = %s;\n" % (name, tmp)

            self.param_set += "if (param.get_name() == " + "\"%s\") {\n" % name
            self.param_set += "%s = param.as_bool();\n" % (nested_name + name)
            self.param_set += "}\n"

        param_prefix = "p_"
        param_prefix += "".join(x + "_" for x in nested_name_list[1:])
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        self.param_declare += "auto %s = rclcpp::ParameterValue(%s);\n" % (param_prefix + name, nested_name + name)
        self.param_declare += "parameters_interface->declare_parameter(\"%s\", %s);\n" % (
            param_name, param_prefix + name)

    def parse_dict(self, name, root_map, nested_name):
        if isinstance(root_map, dict):
            if name != self.target:
                self.struct += "struct %s {\n" % name
            for key in root_map:
                nested_name.append(name)
                self.parse_dict(key, root_map[key], nested_name)
                nested_name.pop()

            if name != self.target:
                self.struct += "} %s_;\n" % name
        else:
            self.parse_params(name, root_map, nested_name)

    def run(self):

        param_gen_directory = sys.argv[0].split("/")
        param_gen_directory = "".join(x + "/" for x in param_gen_directory[:-1])

        out_directory = sys.argv[1]

        if out_directory[-1] != "/":
            out_directory += "/"
        if param_gen_directory[-1] != "/":
            param_gen_directory += "/"

        yaml_file = sys.argv[2]
        self.target = sys.argv[3]

        with open(yaml_file) as f:
            docs = yaml.load_all(f, Loader=yaml.FullLoader)
            if len(sys.argv) != 4:
                print("gen_yaml expects three input argument: directory, yaml file name, and target name")
                raise AssertionError()

            for doc in docs:
                for k, v in doc.items():
                    if k == self.target:
                        self.parse_dict(self.target, v['ros__parameters'], [])

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
        self.contents = self.contents.replace("**DECLARE_PARAMS**", str(self.param_declare))

        if not os.path.isdir(out_directory):
            print("The specified output directory: " + out_directory + " does not exist")
            raise AssertionError()

        with open(out_directory + self.target + ".h", "w") as f:
            f.write(self.contents)


if __name__ == "__main__":
    gen_param_struct = GenParamStruct()
    gen_param_struct.run()
