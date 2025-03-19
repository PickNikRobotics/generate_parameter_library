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
from pathlib import Path


def generate_parameter_module(
    module_name, yaml_file, validation_module='', install_base=None, merge_install=False
):
    # There's no nice way of auto-detecting if colcon was run with --merge-install or not
    # Have to manually specify if using merge install

    if len(sys.argv) <= 2:
        return

    # Get python version
    tmp = sys.version.split()[0]
    tmp = tmp.split('.')
    py_version = f'python{tmp[0]}.{tmp[1]}'

    # Get pkg name and colcon_ws
    colcon_ws, pkg_name = None, None
    for i, arg in enumerate(sys.argv):
        if arg == '--build-directory' or arg == '--build-base':
            build_arg = Path(sys.argv[i + 1])
            colcon_ws = build_arg.parent.parent.parent
            pkg_name = build_arg.parent.stem

    cwd = Path(os.getcwd())
    colcon_ws = colcon_ws if colcon_ws else str(cwd.parent.parent)
    pkg_name = pkg_name if pkg_name else cwd.stem

    # Get paths
    if not install_base:
        install_base = os.path.join(colcon_ws, 'install')

    install_base = (
        install_base if merge_install else os.path.join(install_base, pkg_name)
    )
    install_lib = None
    for i, arg in enumerate(sys.argv):
        if arg == '--install-lib':
            install_lib = sys.argv[i + 1]
            install_lib_split = install_lib.split('/')
            for i, val in enumerate(install_lib_split):
                if '$' in val:
                    env_val = os.getenv(val)
                    install_lib_split[i] = env_val if env_val else ''

    # Get final install and build directories
    install_dir = os.path.join(
        install_base,
        'lib',
        py_version,
        'site-packages',
        pkg_name,
    )
    build_dir = os.path.join(colcon_ws, 'build', pkg_name, pkg_name)

    # Auto-generate python script
    run(os.path.join(build_dir, module_name + '.py'), yaml_file, validation_module)
    run(os.path.join(install_dir, module_name + '.py'), yaml_file, validation_module)
