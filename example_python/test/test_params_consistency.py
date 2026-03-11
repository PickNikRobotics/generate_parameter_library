#!/usr/bin/env python3

# Copyright 2024 PickNik Inc.
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

"""
Tests that verify consistency between get_params() (library internal state)
and node.get_parameter() (ROS2 parameter server) for admittance_controller.

After ParamListener.__init__() calls declare_params(), all parameters (static
and dynamic map entries) are declared in the ROS2 node and read back via
get_parameter(), then stored in params_ via update_internal_params(). This
test verifies both sources remain in sync under default values and after
updates via set_parameters().
"""

import importlib
import math
import unittest

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import generate_parameter_module_example.admittance_parameters


class TestParamsConsistency(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Reload the module to reset class-level attribute state between tests.
        # See: https://github.com/PickNikRobotics/generate_parameter_library/issues/313
        importlib.reload(generate_parameter_module_example.admittance_parameters)
        from generate_parameter_module_example.admittance_parameters import admittance_controller as ac
        self.ac = ac

        # Provide values for required parameters (declared with type-only, no default).
        # Without these, declare_params() raises ParameterUninitializedException when
        # calling get_parameter() on them after declare_parameter(type_only).
        required_params = [
            Parameter('fixed_string_no_default', value='happy'),
            Parameter('command_interfaces', value=['position']),
            Parameter('state_interfaces', value=['position', 'velocity']),
            Parameter('chainable_command_interfaces', value=['position', 'velocity']),
            Parameter('kinematics.plugin_name', value='kdl_plugin/KDLKinematics'),
            Parameter('kinematics.plugin_package', value='kinematics_interface_kdl'),
            Parameter('kinematics.base', value='base_link'),
            Parameter('kinematics.tip', value='ee_link'),
            Parameter('kinematics.group_name', value='manipulator'),
            Parameter('ft_sensor.name', value='tcp_fts_sensor'),
            Parameter('ft_sensor.frame.id', value='ee_link'),
            Parameter('control.frame.id', value='ee_link'),
            Parameter('fixed_world_frame.frame.id', value='base_link'),
            Parameter('gravity_compensation.frame.id', value='ee_link'),
            Parameter('gravity_compensation.CoG.pos', value=[0.1, 0.0, 0.0]),
            Parameter('admittance.selected_axes', value=[True] * 6),
            Parameter('admittance.mass', value=[3.0] * 6),
            Parameter('admittance.damping_ratio', value=[2.828427] * 6),
            Parameter('admittance.stiffness', value=[50.0, 50.0, 50.0, 1.0, 1.0, 1.0]),
        ]
        self.node = Node(
            'test_admittance_controller',
            automatically_declare_parameters_from_overrides=True,
            parameter_overrides=required_params,
        )
        self.listener = self.ac.ParamListener(self.node)
        self.params = self.listener.get_params()

    def tearDown(self):
        self.node.destroy_node()

    # -------------------------------------------------------------------------
    # Static / flat parameters
    # -------------------------------------------------------------------------

    def test_scientific_notation_num(self):
        self.assertAlmostEqual(
            self.params.scientific_notation_num,
            self.node.get_parameter('scientific_notation_num').value,
        )

    def test_interpolation_mode(self):
        self.assertEqual(
            self.params.interpolation_mode,
            self.node.get_parameter('interpolation_mode').value,
        )

    def test_subset_selection(self):
        self.assertEqual(
            list(self.params.subset_selection),
            list(self.node.get_parameter('subset_selection').value),
        )

    def test_joints(self):
        self.assertEqual(
            list(self.params.joints),
            list(self.node.get_parameter('joints').value),
        )

    def test_dof_names(self):
        self.assertEqual(
            list(self.params.dof_names),
            list(self.node.get_parameter('dof_names').value),
        )

    def test_fixed_string(self):
        self.assertEqual(
            self.params.fixed_string,
            self.node.get_parameter('fixed_string').value,
        )

    def test_fixed_array(self):
        self.assertEqual(
            list(self.params.fixed_array),
            list(self.node.get_parameter('fixed_array').value),
        )

    def test_enable_parameter_update_without_reactivation(self):
        self.assertEqual(
            self.params.enable_parameter_update_without_reactivation,
            self.node.get_parameter('enable_parameter_update_without_reactivation').value,
        )

    def test_use_feedforward_commanded_input(self):
        self.assertEqual(
            self.params.use_feedforward_commanded_input,
            self.node.get_parameter('use_feedforward_commanded_input').value,
        )

    def test_lt_eq_fifteen(self):
        self.assertEqual(
            self.params.lt_eq_fifteen,
            self.node.get_parameter('lt_eq_fifteen').value,
        )

    def test_gt_fifteen(self):
        self.assertEqual(
            self.params.gt_fifteen,
            self.node.get_parameter('gt_fifteen').value,
        )

    def test_one_number(self):
        self.assertEqual(
            self.params.one_number,
            self.node.get_parameter('one_number').value,
        )

    def test_three_numbers(self):
        self.assertEqual(
            list(self.params.three_numbers),
            list(self.node.get_parameter('three_numbers').value),
        )

    def test_three_numbers_of_five(self):
        self.assertEqual(
            list(self.params.three_numbers_of_five),
            list(self.node.get_parameter('three_numbers_of_five').value),
        )

    def test_hover_override(self):
        self.assertEqual(
            self.params.hover_override,
            self.node.get_parameter('hover_override').value,
        )

    def test_angle_wraparound(self):
        self.assertEqual(
            self.params.angle_wraparound,
            self.node.get_parameter('angle_wraparound').value,
        )

    def test_open_loop_control(self):
        self.assertEqual(
            self.params.open_loop_control,
            self.node.get_parameter('open_loop_control').value,
        )

    # -------------------------------------------------------------------------
    # Nested struct parameters
    # -------------------------------------------------------------------------

    def test_pid_rate(self):
        self.assertAlmostEqual(
            self.params.pid.rate,
            self.node.get_parameter('pid.rate').value,
        )

    def test_kinematics_alpha(self):
        self.assertAlmostEqual(
            self.params.kinematics.alpha,
            self.node.get_parameter('kinematics.alpha').value,
        )

    def test_ft_sensor_filter_coefficient(self):
        self.assertAlmostEqual(
            self.params.ft_sensor.filter_coefficient,
            self.node.get_parameter('ft_sensor.filter_coefficient').value,
        )

    def test_ft_sensor_frame_external(self):
        self.assertEqual(
            self.params.ft_sensor.frame.external,
            self.node.get_parameter('ft_sensor.frame.external').value,
        )

    def test_control_frame_external(self):
        self.assertEqual(
            self.params.control.frame.external,
            self.node.get_parameter('control.frame.external').value,
        )

    def test_fixed_world_frame_external(self):
        self.assertEqual(
            self.params.fixed_world_frame.frame.external,
            self.node.get_parameter('fixed_world_frame.frame.external').value,
        )

    def test_gravity_compensation_frame_external(self):
        self.assertEqual(
            self.params.gravity_compensation.frame.external,
            self.node.get_parameter('gravity_compensation.frame.external').value,
        )

    def test_gravity_compensation_cog_force_is_nan(self):
        self.assertTrue(math.isnan(self.params.gravity_compensation.CoG.force))
        self.assertTrue(
            math.isnan(self.node.get_parameter('gravity_compensation.CoG.force').value)
        )

    # -------------------------------------------------------------------------
    # Dynamic map parameters
    # -------------------------------------------------------------------------

    def test_map_joints_dof_weight(self):
        for joint in self.params.joints:
            for dof in self.params.dof_names:
                param_name = f'{joint}.{dof}.weight'
                lib_value = self.params.get_entry(joint).get_entry(dof).weight
                ros_value = self.node.get_parameter(param_name).value
                self.assertAlmostEqual(
                    lib_value, ros_value,
                    msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
                )

    def test_nested_dynamic_map(self):
        for joint in self.params.joints:
            for dof in self.params.dof_names:
                param_name = f'nested_dynamic.{joint}.{dof}.nested'
                lib_value = self.params.nested_dynamic.get_entry(joint).get_entry(dof).nested
                ros_value = self.node.get_parameter(param_name).value
                self.assertAlmostEqual(
                    lib_value, ros_value,
                    msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
                )

    def test_pid_map_joints_p(self):
        for joint in self.params.joints:
            param_name = f'pid.{joint}.p'
            lib_value = self.params.pid.get_entry(joint).p
            ros_value = self.node.get_parameter(param_name).value
            self.assertAlmostEqual(
                lib_value, ros_value,
                msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
            )

    def test_pid_map_joints_i(self):
        for joint in self.params.joints:
            param_name = f'pid.{joint}.i'
            lib_value = self.params.pid.get_entry(joint).i
            ros_value = self.node.get_parameter(param_name).value
            self.assertAlmostEqual(
                lib_value, ros_value,
                msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
            )

    def test_pid_map_joints_d(self):
        for joint in self.params.joints:
            param_name = f'pid.{joint}.d'
            lib_value = self.params.pid.get_entry(joint).d
            ros_value = self.node.get_parameter(param_name).value
            self.assertAlmostEqual(
                lib_value, ros_value,
                msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
            )

    def test_gains_map_dof_k(self):
        for dof in self.params.dof_names:
            param_name = f'gains.{dof}.k'
            lib_value = self.params.gains.get_entry(dof).k
            ros_value = self.node.get_parameter(param_name).value
            self.assertAlmostEqual(
                lib_value, ros_value,
                msg=f'Mismatch for {param_name}: get_params()={lib_value}, get_parameter()={ros_value}',
            )

    # -------------------------------------------------------------------------
    # Consistency after set_parameters()
    # -------------------------------------------------------------------------

    def test_update_interpolation_mode(self):
        new_value = 'linear'
        self.node.set_parameters([Parameter('interpolation_mode', value=new_value)])
        updated_params = self.listener.get_params()
        self.assertEqual(updated_params.interpolation_mode, new_value)
        self.assertEqual(
            updated_params.interpolation_mode,
            self.node.get_parameter('interpolation_mode').value,
        )

    def test_update_lt_eq_fifteen(self):
        new_value = 10
        self.node.set_parameters([Parameter('lt_eq_fifteen', value=new_value)])
        updated_params = self.listener.get_params()
        self.assertEqual(updated_params.lt_eq_fifteen, new_value)
        self.assertEqual(
            updated_params.lt_eq_fifteen,
            self.node.get_parameter('lt_eq_fifteen').value,
        )

    def test_update_map_joint_dof_weight(self):
        joint = self.params.joints[0]
        dof = self.params.dof_names[0]
        param_name = f'{joint}.{dof}.weight'
        new_value = 2.5
        self.node.set_parameters([Parameter(param_name, value=new_value)])
        updated_params = self.listener.get_params()
        lib_value = updated_params.get_entry(joint).get_entry(dof).weight
        ros_value = self.node.get_parameter(param_name).value
        self.assertAlmostEqual(lib_value, new_value)
        self.assertAlmostEqual(lib_value, ros_value)


def main():
    unittest.main()


if __name__ == '__main__':
    main()
