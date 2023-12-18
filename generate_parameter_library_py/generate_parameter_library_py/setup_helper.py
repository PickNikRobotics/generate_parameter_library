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

import sys
import os
from generate_parameter_library_py.generate_python_module import run


def generate_parameter_module(module_name, yaml_file, validation_module=''):
    # TODO there must be a better way to do this. I need to find the build directory so I can place the python
    # module there
    build_dir = None
    install_dir = None
    for i, arg in enumerate(sys.argv):
        # Look for the `--build-directory` option in the command line arguments
        if arg == '--build-directory' or arg == '--build-base':
            build_arg = sys.argv[i + 1]

            path_split = os.path.split(build_arg)
            path_split = os.path.split(path_split[0])
            pkg_name = path_split[1]
            path_split = os.path.split(path_split[0])
            colcon_ws = path_split[0]

            tmp = sys.version.split()[0]
            tmp = tmp.split('.')
            py_version = f'python{tmp[0]}.{tmp[1]}'

            install_dir = os.path.join(
                colcon_ws,
                'install',
                pkg_name,
                'lib',
                py_version,
                'site-packages',
                pkg_name,
            )
            build_dir = os.path.join(colcon_ws, 'build', pkg_name, pkg_name)
            break

    if build_dir:
        run(os.path.join(build_dir, module_name + '.py'), yaml_file, validation_module)
    if install_dir:
        run(
            os.path.join(install_dir, module_name + '.py'), yaml_file, validation_module
        )
