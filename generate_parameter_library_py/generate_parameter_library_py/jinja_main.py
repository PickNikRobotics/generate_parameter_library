from jinja2 import Template
from os import listdir
from os.path import isfile, join

template = """hostname {{ hostname }}

no ip domain lookup
ip domain name local.lab
ip name-server {{ name_server_pri }}
ip name-server {{ name_server_sec }}

ntp server {{ ntp_server_pri }} prefer
ntp server {{ ntp_server_sec }}"""

data = {
    "conditional": "1 > 2",
}

if __name__ == "__main__":
    root = "/home/paul/Downloads/colcon_ws/src/generate_parameter_library/generate_parameter_library_py"
    mypath = join(root, "generate_parameter_library_py", "jinja_templates")
    template_map = dict()
    for file_name in [f for f in listdir(mypath) if isfile(join(mypath, f))]:
        with open(join(mypath, file_name)) as file:
            template_map[file_name] = file.read()

    j2_template = Template(template_map['if_statement'])
    tmp = j2_template.render(data)
    print(tmp)
