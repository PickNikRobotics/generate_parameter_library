# Copyright 2026 Visionick
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest

from generate_parameter_library_py.parse_yaml import get_dynamic_mapped_parameter


@pytest.mark.parametrize(
    'param_name,declared_params,expected',
    [
        # entries is inside nested -> resolve to nested.entries
        ('nested.__map_entries.value', {'nested.entries'}, ['nested.entries']),
        # items is inside level1.level2 -> resolve to level1.level2.items
        (
            'level1.level2.__map_items.param',
            {'level1.level2.items'},
            ['level1.level2.items'],
        ),
        # keys is at root -> resolve to bare name
        ('__map_keys.value', {'keys'}, ['keys']),
        # multi-level maps with keys at root -> all bare names
        (
            '__map_level1.__map_level2.__map_level3.value',
            {'level1', 'level2', 'level3'},
            ['level1', 'level2', 'level3'],
        ),
        # keys at root but __map_ inside a struct (like pid.__map_joints)
        ('pid.__map_joints.p', {'joints'}, ['joints']),
        # nested struct with keys at root (like nested_dynamic.__map_joints)
        (
            'nested_dynamic.__map_joints.__map_dof_names.nested',
            {'joints', 'dof_names'},
            ['joints', 'dof_names'],
        ),
    ],
)
def test_get_dynamic_mapped_parameter_nested(param_name, declared_params, expected):
    """Test that get_dynamic_mapped_parameter returns correct paths for nested maps."""
    result = get_dynamic_mapped_parameter(param_name, declared_params)
    assert result == expected
