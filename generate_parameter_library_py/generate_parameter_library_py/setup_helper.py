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


def _get_option_value(args, option_names):
    for i, arg in enumerate(args):
        for option_name in option_names:
            # Handle `--opt value` and `--opt=value` forms.
            if arg == option_name and i + 1 < len(args):
                return args[i + 1]
            if arg.startswith(option_name + '='):
                return arg.split('=', 1)[1]
    return None


def _append_unique(paths, path):
    if path and path not in paths:
        paths.append(path)


def _package_name_from_yaml_path(yaml_file):
    normalized = yaml_file.replace('\\', '/')
    return normalized.split('/', 1)[0]


def _is_within(path, parent):
    try:
        return os.path.commonpath([path, parent]) == parent
    except ValueError:
        return False


def _source_package_dir(package_name):
    package_path = os.path.join(os.getcwd(), package_name)
    if os.path.isdir(package_path):
        return os.path.realpath(package_path)
    return None


def generate_parameter_module(
    module_name, yaml_file, validation_module='', install_base=None, merge_install=False
):
    package_name = _package_name_from_yaml_path(yaml_file)
    output_dirs = []
    src_pkg_dir = _source_package_dir(package_name)

    tmp = sys.version.split()[0]
    tmp = tmp.split('.')
    py_version = f'python{tmp[0]}.{tmp[1]}'

    build_arg = _get_option_value(sys.argv, ['--build-directory', '--build-base'])
    if build_arg:
        # Typical colcon editable invocation uses --build-directory.
        path_split = os.path.split(build_arg)
        path_split = os.path.split(path_split[0])
        pkg_name = path_split[1]
        path_split = os.path.split(path_split[0])
        colcon_ws = path_split[0]

        if not install_base:
            install_base = os.path.join(colcon_ws, 'install')

        resolved_install_base = (
            install_base if merge_install else os.path.join(install_base, pkg_name)
        )
        _append_unique(
            output_dirs,
            os.path.join(colcon_ws, 'build', pkg_name, pkg_name),
        )
        _append_unique(
            output_dirs,
            os.path.join(
                resolved_install_base,
                'lib',
                py_version,
                'site-packages',
                pkg_name,
            ),
        )

    # Handle standard setuptools/distutils build and install options used on
    # buildfarm and by non-editable local builds.
    build_lib = _get_option_value(sys.argv, ['--build-lib'])
    if build_lib:
        _append_unique(output_dirs, os.path.join(build_lib, package_name))

    install_lib = _get_option_value(sys.argv, ['--install-lib'])
    if install_lib:
        install_lib_dir = install_lib
        if os.path.basename(install_lib) != package_name:
            install_lib_dir = os.path.join(install_lib, package_name)
        _append_unique(output_dirs, install_lib_dir)

    prefix = _get_option_value(sys.argv, ['--prefix'])
    if prefix:
        root = _get_option_value(sys.argv, ['--root'])
        prefix_base = prefix
        if root:
            prefix_base = os.path.join(root, prefix.lstrip(os.sep))
        _append_unique(
            output_dirs,
            os.path.join(
                prefix_base,
                'lib',
                py_version,
                'site-packages',
                package_name,
            ),
        )

    for output_dir in output_dirs:
        real_output_dir = os.path.realpath(output_dir)
        # Avoid writing generated artifacts into source trees. This can happen
        # in editable/symlink builds where build paths resolve to source.
        if src_pkg_dir and _is_within(real_output_dir, src_pkg_dir):
            continue
        run(
            os.path.join(output_dir, module_name + '.py'),
            yaml_file,
            validation_module,
        )
