#!/usr/bin/env python3

import yaml
import sys
import os

# class used to fill template.txt using passed in yaml file
class GenParamStruct:

    def __init__(self):
        self.contents = ""
        self.struct = ""
        self.param_set = ""
        self.param_declare = ""
        self.target = ""

    def parse_params(self, name, value, nested_name_list):

        nested_name = "".join(x + "_." for x in nested_name_list[1:])

        if isinstance(value, list) and isinstance(value[0], str):
            self.struct += "std::vector<std::string> " + name +" = {"
            for ind, val in enumerate(value):
                if ind < len(value)-1:
                    self.struct += "\"" + val + "\"" + ", "
                else:
                    self.struct += "\"" + val + "\""  + "};\n"

            self.param_set += "if (param.get_name() == " + "\"" + name + "\") {\n"
            self.param_set += nested_name + name + " = param.as_string_array();\n"
            self.param_set += "}\n"

        elif isinstance(value, list) and isinstance(value[0], float):
            self.struct += "std::vector<double> " + name +" = {"
            for ind, val in enumerate(value):
                if ind < len(value)-1:
                    self.struct += str(val) + ", "
                else:
                    self.struct += str(val) + "};\n"

            self.param_set += "if (param.get_name() == " + "\"" + name + "\") {\n"
            self.param_set += nested_name + name + " = param.as_double_array();\n"
            self.param_set += "}\n"

        elif isinstance(value, str):
            self.struct += "std::string " + name + " = \"" + value + "\";\n"

            self.param_set += "if (param.get_name() == " + "\"" + name + "\") {\n"
            self.param_set += nested_name + name + " = param.as_string();\n"
            self.param_set += "}\n"

        elif isinstance(value, float):
            self.struct += "double " + name + " = " + str(value)+";\n"

            self.param_set += "if (param.get_name() == " + "\"" + name + "\") {\n"
            self.param_set += nested_name + name + " = param.as_double();\n"
            self.param_set += "}\n"

        elif isinstance(value, bool):
            if value:
                tmp = "true"
            else:
                tmp = "false"
            self.struct += "bool " + name + " = " + tmp + ";\n"

            self.param_set += "if (param.get_name() == " + "\"" + name + "\") {\n"
            self.param_set += nested_name + name + " = param.as_bool();\n"
            self.param_set += "}\n"

        param_prefix = "p_" + "".join(x + "_" for x in nested_name_list[1:])
        param_name = "".join(x + "." for x in nested_name_list[1:]) + name

        self.param_declare += "auto " + param_prefix + name + " = rclcpp::ParameterValue(" + nested_name + name + ");\n"
        self.param_declare += "parameters_interface->declare_parameter(\"" + param_name + "\", " + param_prefix + name +");\n"

    def parse_dict(self, name, root_map, nested_name):
        if isinstance(root_map, dict):
            if name != self.target:
                self.struct += "struct "+name+" {\n"
            for key in root_map:
                # if name != self.target:
                nested_name.append(name)
                self.parse_dict(key, root_map[key], nested_name)
                nested_name.pop()

            if name != self.target:
                self.struct += "} " + name + "_;\n"
        else:
            self.parse_params(name, root_map, nested_name)

    def run(self):

        directory = sys.argv[1]
        yaml_file = sys.argv[2]
        self.target = sys.argv[3]

        with open(directory + "/" + yaml_file) as f:
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

        with open(directory + "/templates/template.txt", "r") as f:
            f = open(directory + "/templates/template.txt", "r")
            self.contents = f.read()

        self.contents = self.contents.replace("**COMMENTS**", COMMENTS)
        self.contents = self.contents.replace("**INCLUDES**", INCLUDES)
        self.contents = self.contents.replace("**NAMESPACE**", NAMESPACE)
        self.contents = self.contents.replace("**STRUCT_NAME**", self.target)
        self.contents = self.contents.replace("**STRUCT_CONTENT**", self.struct)
        self.contents = self.contents.replace("**PARAM_SET**", self.param_set)
        self.contents = self.contents.replace("**DECLARE_PARAMS**", self.param_declare)

        paths = [directory + "/include/", directory + "/include/parameters/"]
        for path in paths:
            if not os.path.isdir(path):
                os.mkdir(path)

        with open(paths[-1] + self.target  + ".h", "w") as f:
            f.write(self.contents)



if __name__ == "__main__":
    gen_param_struct = GenParamStruct()
    gen_param_struct.run()