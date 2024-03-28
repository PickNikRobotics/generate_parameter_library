# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators

import generate_parameter_module_example.custom_validation as custom_validators


class admittance_controller:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        scientific_notation_num = 1e-11
        interpolation_mode = "spline"
        subset_selection = ["A", "B"]
        joints = ["joint1", "joint2", "joint3"]
        dof_names = ["x", "y", "rz"]
        fixed_string = "string_value"
        fixed_array = [1.0, 2.3, 4.0, 5.4, 3.3]
        fixed_string_no_default = None
        command_interfaces = None
        state_interfaces = None
        chainable_command_interfaces = None
        enable_parameter_update_without_reactivation = True
        use_feedforward_commanded_input = False
        lt_eq_fifteen = 1
        gt_fifteen = 16
        one_number = 14540
        three_numbers = [3, 4, 5]
        three_numbers_of_five = [3, 3, 3]
        hover_override = 1
        angle_wraparound = False
        open_loop_control = False
        class __MapJoints:
            class __MapDofNames:
                weight = 1.0
            __map_type = __MapDofNames
            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())
                return getattr(self, name)
            def get_entry(self, name):
                return getattr(self, name)
        __map_type = __MapJoints
        def add_entry(self, name):
            if not hasattr(self, name):
                setattr(self, name, self.__map_type())
            return getattr(self, name)
        def get_entry(self, name):
            return getattr(self, name)
        class __NestedDynamic:
            class __MapJoints:
                class __MapDofNames:
                    nested = 1.0
                    class __MapJoints:
                        class __MapDofNames:
                            nested_deep = 1.0
                        __map_type = __MapDofNames
                        def add_entry(self, name):
                            if not hasattr(self, name):
                                setattr(self, name, self.__map_type())
                            return getattr(self, name)
                        def get_entry(self, name):
                            return getattr(self, name)
                    __map_type = __MapJoints
                    def add_entry(self, name):
                        if not hasattr(self, name):
                            setattr(self, name, self.__map_type())
                        return getattr(self, name)
                    def get_entry(self, name):
                        return getattr(self, name)
                __map_type = __MapDofNames
                def add_entry(self, name):
                    if not hasattr(self, name):
                        setattr(self, name, self.__map_type())
                    return getattr(self, name)
                def get_entry(self, name):
                    return getattr(self, name)
            __map_type = __MapJoints
            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())
                return getattr(self, name)
            def get_entry(self, name):
                return getattr(self, name)
        nested_dynamic = __NestedDynamic()
        class __Pid:
            rate = 0.005
            class __MapJoints:
                p = 1.0
                i = 1.0
                d = 1.0
            __map_type = __MapJoints
            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())
                return getattr(self, name)
            def get_entry(self, name):
                return getattr(self, name)
        pid = __Pid()
        class __Gains:
            class __MapDofNames:
                k = 2.0
            __map_type = __MapDofNames
            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())
                return getattr(self, name)
            def get_entry(self, name):
                return getattr(self, name)
        gains = __Gains()
        class __Kinematics:
            plugin_name = None
            plugin_package = None
            base = None
            tip = None
            alpha = 0.0005
            group_name = None
        kinematics = __Kinematics()
        class __FtSensor:
            name = None
            filter_coefficient = 0.005
            class __Frame:
                id = None
                external = False
            frame = __Frame()
        ft_sensor = __FtSensor()
        class __Control:
            class __Frame:
                id = None
                external = False
            frame = __Frame()
        control = __Control()
        class __FixedWorldFrame:
            class __Frame:
                id = None
                external = False
            frame = __Frame()
        fixed_world_frame = __FixedWorldFrame()
        class __GravityCompensation:
            class __Frame:
                id = None
                external = False
            frame = __Frame()
            class __Cog:
                pos = None
                force = float('nan')
            CoG = __Cog()
        gravity_compensation = __GravityCompensation()
        class __Admittance:
            selected_axes = None
            mass = None
            damping_ratio = None
            stiffness = None
        admittance = __Admittance()



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = admittance_controller.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("admittance_controller." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


            for value_1 in updated_params.joints:


                for value_2 in updated_params.dof_names:


                    updated_params.add_entry(value_1).add_entry(value_2)
                    entry = updated_params.get_entry(value_1).get_entry(value_2)
                    param_name = f"{self.prefix_}{value_1}.{value_2}.weight"
                    if not self.node_.has_parameter(self.prefix_ + param_name):
                        descriptor = ParameterDescriptor(description="map parameter without struct name", read_only = False)
                        descriptor.floating_point_range.append(FloatingPointRange())
                        descriptor.floating_point_range[-1].from_value = 0.0
                        descriptor.floating_point_range[-1].to_value = float('inf')
                        parameter = entry.weight
                        self.node_.declare_parameter(param_name, parameter, descriptor)
                    param = self.node_.get_parameter(param_name)
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        raise InvalidParameterValueException('__map_joints.__map_dof_names.weight',param.value, 'Invalid value set during initialization for parameter __map_joints.__map_dof_names.weight: ' + validation_result)
                    entry.weight = param.value


            for value_1 in updated_params.nested_dynamic.joints:


                for value_2 in updated_params.nested_dynamic.dof_names:


                    updated_params.nested_dynamic.add_entry(value_1).add_entry(value_2)
                    entry = updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2)
                    param_name = f"{self.prefix_}nested_dynamic.{value_1}.{value_2}.nested"
                    if not self.node_.has_parameter(self.prefix_ + param_name):
                        descriptor = ParameterDescriptor(description="test nested map params", read_only = False)
                        descriptor.floating_point_range.append(FloatingPointRange())
                        descriptor.floating_point_range[-1].from_value = 0.0001
                        descriptor.floating_point_range[-1].to_value = float('inf')
                        parameter = entry.nested
                        self.node_.declare_parameter(param_name, parameter, descriptor)
                    param = self.node_.get_parameter(param_name)
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        raise InvalidParameterValueException('nested_dynamic.__map_joints.__map_dof_names.nested',param.value, 'Invalid value set during initialization for parameter nested_dynamic.__map_joints.__map_dof_names.nested: ' + validation_result)
                    entry.nested = param.value


            for value_1 in updated_params.nested_dynamic.joints:


                for value_2 in updated_params.nested_dynamic.dof_names:


                    for value_3 in updated_params.nested_dynamic.joints:


                        for value_4 in updated_params.nested_dynamic.dof_names:


                            updated_params.nested_dynamic.add_entry(value_1).add_entry(value_2).add_entry(value_3).add_entry(value_4)
                            entry = updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2).get_entry(value_3).get_entry(value_4)
                            param_name = f"{self.prefix_}nested_dynamic.{value_1}.{value_2}.{value_3}.{value_4}.nested_deep"
                            if not self.node_.has_parameter(self.prefix_ + param_name):
                                descriptor = ParameterDescriptor(description="test deep nested map params", read_only = False)
                                descriptor.floating_point_range.append(FloatingPointRange())
                                descriptor.floating_point_range[-1].from_value = 0.0001
                                descriptor.floating_point_range[-1].to_value = float('inf')
                                parameter = entry.nested_deep
                                self.node_.declare_parameter(param_name, parameter, descriptor)
                            param = self.node_.get_parameter(param_name)
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                            validation_result = ParameterValidators.gt_eq(param, 0.0001)
                            if validation_result:
                                raise InvalidParameterValueException('nested_dynamic.__map_joints.__map_dof_names.__map_joints.__map_dof_names.nested_deep',param.value, 'Invalid value set during initialization for parameter nested_dynamic.__map_joints.__map_dof_names.__map_joints.__map_dof_names.nested_deep: ' + validation_result)
                            entry.nested_deep = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.p"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="proportional gain term", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0001
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.p
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0.0001)
                if validation_result:
                    raise InvalidParameterValueException('pid.__map_joints.p',param.value, 'Invalid value set during initialization for parameter pid.__map_joints.p: ' + validation_result)
                entry.p = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.i"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="integral gain term", read_only = False)
                    parameter = entry.i
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.i = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.d"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="derivative gain term", read_only = False)
                    parameter = entry.d
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.d = param.value


            for value_1 in updated_params.gains.dof_names:


                updated_params.gains.add_entry(value_1)
                entry = updated_params.gains.get_entry(value_1)
                param_name = f"{self.prefix_}gains.{value_1}.k"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="general gain", read_only = False)
                    parameter = entry.k
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.k = param.value

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "scientific_notation_num":
                    updated_params.scientific_notation_num = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "interpolation_mode":
                    validation_result = ParameterValidators.one_of(param, ["spline", "linear"])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = custom_validators.no_args_validator(param)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.interpolation_mode = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "subset_selection":
                    validation_result = ParameterValidators.subset_of(param, ["A", "B", "C"])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.subset_selection = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "joints":
                    updated_params.joints = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "dof_names":
                    updated_params.dof_names = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "pid.rate":
                    updated_params.pid.rate = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "fixed_string":
                    validation_result = ParameterValidators.size_lt(param, 26)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.fixed_string = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "fixed_array":
                    validation_result = ParameterValidators.size_lt(param, 11)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.fixed_array = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "fixed_string_no_default":
                    validation_result = ParameterValidators.size_lt(param, 26)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.fixed_string_no_default = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "command_interfaces":
                    updated_params.command_interfaces = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "state_interfaces":
                    updated_params.state_interfaces = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "chainable_command_interfaces":
                    updated_params.chainable_command_interfaces = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.plugin_name":
                    updated_params.kinematics.plugin_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.plugin_package":
                    updated_params.kinematics.plugin_package = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.base":
                    updated_params.kinematics.base = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.tip":
                    updated_params.kinematics.tip = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.alpha":
                    updated_params.kinematics.alpha = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "kinematics.group_name":
                    updated_params.kinematics.group_name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ft_sensor.name":
                    updated_params.ft_sensor.name = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ft_sensor.frame.id":
                    updated_params.ft_sensor.frame.id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ft_sensor.frame.external":
                    updated_params.ft_sensor.frame.external = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "ft_sensor.filter_coefficient":
                    updated_params.ft_sensor.filter_coefficient = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "control.frame.id":
                    updated_params.control.frame.id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "control.frame.external":
                    updated_params.control.frame.external = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "fixed_world_frame.frame.id":
                    updated_params.fixed_world_frame.frame.id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "fixed_world_frame.frame.external":
                    updated_params.fixed_world_frame.frame.external = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_compensation.frame.id":
                    updated_params.gravity_compensation.frame.id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_compensation.frame.external":
                    updated_params.gravity_compensation.frame.external = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_compensation.CoG.pos":
                    validation_result = ParameterValidators.fixed_size(param, 3)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.gravity_compensation.CoG.pos = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gravity_compensation.CoG.force":
                    updated_params.gravity_compensation.CoG.force = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "admittance.selected_axes":
                    validation_result = ParameterValidators.fixed_size(param, 6)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.admittance.selected_axes = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "admittance.mass":
                    validation_result = ParameterValidators.fixed_size(param, 6)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.element_bounds(param, 0.0001, 1000000.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.admittance.mass = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "admittance.damping_ratio":
                    validation_result = ParameterValidators.fixed_size(param, 6)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = custom_validators.validate_double_array_custom_func(param, 20.3, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    validation_result = ParameterValidators.element_bounds(param, 0.1, 10.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.admittance.damping_ratio = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "admittance.stiffness":
                    validation_result = ParameterValidators.element_bounds(param, 0.0001, 100000.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.admittance.stiffness = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "enable_parameter_update_without_reactivation":
                    updated_params.enable_parameter_update_without_reactivation = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "use_feedforward_commanded_input":
                    updated_params.use_feedforward_commanded_input = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "lt_eq_fifteen":
                    validation_result = ParameterValidators.lt_eq(param, 15)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.lt_eq_fifteen = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "gt_fifteen":
                    validation_result = ParameterValidators.gt(param, 15)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.gt_fifteen = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "one_number":
                    validation_result = ParameterValidators.bounds(param, 1024, 65535)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.one_number = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "three_numbers":
                    updated_params.three_numbers = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "three_numbers_of_five":
                    validation_result = ParameterValidators.size_lt(param, 6)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.three_numbers_of_five = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "hover_override":
                    validation_result = ParameterValidators.one_of(param, [0, 1, 2, -1])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.hover_override = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "angle_wraparound":
                    updated_params.angle_wraparound = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "open_loop_control":
                    updated_params.open_loop_control = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


            # update dynamic parameters
            for param in parameters:


                    for value_1 in updated_params.joints:


                        for value_2 in updated_params.dof_names:


                            param_name = f"{self.prefix_}{value_1}.{value_2}.weight"
                            if param.name == param_name:
                                validation_result = ParameterValidators.gt(param, 0.0)
                                if validation_result:
                                    return SetParametersResult(successful=False, reason=validation_result)

                                updated_params.get_entry(value_1).get_entry(value_2).weight = param.value
                                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.nested_dynamic.joints:


                        for value_2 in updated_params.nested_dynamic.dof_names:


                            param_name = f"{self.prefix_}nested_dynamic{value_1}.{value_2}.nested"
                            if param.name == param_name:
                                validation_result = ParameterValidators.gt_eq(param, 0.0001)
                                if validation_result:
                                    return SetParametersResult(successful=False, reason=validation_result)

                                updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2).nested = param.value
                                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.nested_dynamic.joints:


                        for value_2 in updated_params.nested_dynamic.dof_names:


                            for value_3 in updated_params.nested_dynamic.joints:


                                for value_4 in updated_params.nested_dynamic.dof_names:


                                    param_name = f"{self.prefix_}nested_dynamic{value_1}.{value_2}.{value_3}.{value_4}.nested_deep"
                                    if param.name == param_name:
                                        validation_result = ParameterValidators.gt_eq(param, 0.0001)
                                        if validation_result:
                                            return SetParametersResult(successful=False, reason=validation_result)

                                        updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2).get_entry(value_3).get_entry(value_4).nested_deep = param.value
                                        self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.pid.joints:


                        param_name = f"{self.prefix_}pid{value_1}.p"
                        if param.name == param_name:
                            validation_result = ParameterValidators.gt_eq(param, 0.0001)
                            if validation_result:
                                return SetParametersResult(successful=False, reason=validation_result)

                            updated_params.pid.get_entry(value_1).p = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.pid.joints:


                        param_name = f"{self.prefix_}pid{value_1}.i"
                        if param.name == param_name:

                            updated_params.pid.get_entry(value_1).i = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.pid.joints:


                        param_name = f"{self.prefix_}pid{value_1}.d"
                        if param.name == param_name:

                            updated_params.pid.get_entry(value_1).d = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



                    for value_1 in updated_params.gains.dof_names:


                        param_name = f"{self.prefix_}gains{value_1}.k"
                        if param.name == param_name:

                            updated_params.gains.get_entry(value_1).k = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "scientific_notation_num"):
                descriptor = ParameterDescriptor(description="Test scientific notation", read_only = False)
                parameter = updated_params.scientific_notation_num
                self.node_.declare_parameter(self.prefix_ + "scientific_notation_num", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "interpolation_mode"):
                descriptor = ParameterDescriptor(description="specifies which algorithm to use for interpolation.", read_only = False)
                parameter = updated_params.interpolation_mode
                self.node_.declare_parameter(self.prefix_ + "interpolation_mode", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "subset_selection"):
                descriptor = ParameterDescriptor(description="test subset of validator.", read_only = False)
                parameter = updated_params.subset_selection
                self.node_.declare_parameter(self.prefix_ + "subset_selection", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "joints"):
                descriptor = ParameterDescriptor(description="specifies which joints will be used by the controller", read_only = False)
                parameter = updated_params.joints
                self.node_.declare_parameter(self.prefix_ + "joints", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "dof_names"):
                descriptor = ParameterDescriptor(description="specifies which joints will be used by the controller", read_only = False)
                parameter = updated_params.dof_names
                self.node_.declare_parameter(self.prefix_ + "dof_names", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "pid.rate"):
                descriptor = ParameterDescriptor(description="update loop period in seconds", read_only = False)
                parameter = updated_params.pid.rate
                self.node_.declare_parameter(self.prefix_ + "pid.rate", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "fixed_string"):
                descriptor = ParameterDescriptor(description="test code generation for fixed sized string", read_only = False)
                parameter = updated_params.fixed_string
                self.node_.declare_parameter(self.prefix_ + "fixed_string", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "fixed_array"):
                descriptor = ParameterDescriptor(description="test code generation for fixed sized array", read_only = False)
                parameter = updated_params.fixed_array
                self.node_.declare_parameter(self.prefix_ + "fixed_array", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "fixed_string_no_default"):
                descriptor = ParameterDescriptor(description="test code generation for fixed sized string with no default", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "fixed_string_no_default", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "command_interfaces"):
                descriptor = ParameterDescriptor(description="specifies which command interfaces to claim", read_only = True)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "command_interfaces", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "state_interfaces"):
                descriptor = ParameterDescriptor(description="specifies which state interfaces to claim", read_only = True)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "state_interfaces", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "chainable_command_interfaces"):
                descriptor = ParameterDescriptor(description="specifies which chainable interfaces to claim", read_only = True)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "chainable_command_interfaces", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.plugin_name"):
                descriptor = ParameterDescriptor(description="specifies which kinematics plugin to load", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "kinematics.plugin_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.plugin_package"):
                descriptor = ParameterDescriptor(description="specifies the package to load the kinematics plugin from", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "kinematics.plugin_package", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.base"):
                descriptor = ParameterDescriptor(description="specifies the base link of the robot description used by the kinematics plugin", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "kinematics.base", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.tip"):
                descriptor = ParameterDescriptor(description="specifies the end effector link of the robot description used by the kinematics plugin", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "kinematics.tip", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.alpha"):
                descriptor = ParameterDescriptor(description="specifies the damping coefficient for the Jacobian pseudo inverse", read_only = False)
                parameter = updated_params.kinematics.alpha
                self.node_.declare_parameter(self.prefix_ + "kinematics.alpha", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "kinematics.group_name"):
                descriptor = ParameterDescriptor(description="specifies the group name for planning with Moveit", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "kinematics.group_name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ft_sensor.name"):
                descriptor = ParameterDescriptor(description="name of the force torque sensor in the robot description", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "ft_sensor.name", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ft_sensor.frame.id"):
                descriptor = ParameterDescriptor(description="frame of the force torque sensor", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "ft_sensor.frame.id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ft_sensor.frame.external"):
                descriptor = ParameterDescriptor(description="specifies if the force torque sensor is contained in the kinematics chain from the base to the tip", read_only = False)
                parameter = updated_params.ft_sensor.frame.external
                self.node_.declare_parameter(self.prefix_ + "ft_sensor.frame.external", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "ft_sensor.filter_coefficient"):
                descriptor = ParameterDescriptor(description="specifies the coefficient for the sensor's exponential filter", read_only = False)
                parameter = updated_params.ft_sensor.filter_coefficient
                self.node_.declare_parameter(self.prefix_ + "ft_sensor.filter_coefficient", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "control.frame.id"):
                descriptor = ParameterDescriptor(description="control frame used for admittance control", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "control.frame.id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "control.frame.external"):
                descriptor = ParameterDescriptor(description="specifies if the control frame is contained in the kinematics chain from the base to the tip", read_only = False)
                parameter = updated_params.control.frame.external
                self.node_.declare_parameter(self.prefix_ + "control.frame.external", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "fixed_world_frame.frame.id"):
                descriptor = ParameterDescriptor(description="world frame, gravity points down (neg. Z) in this frame", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "fixed_world_frame.frame.id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "fixed_world_frame.frame.external"):
                descriptor = ParameterDescriptor(description="specifies if the world frame is contained in the kinematics chain from the base to the tip", read_only = False)
                parameter = updated_params.fixed_world_frame.frame.external
                self.node_.declare_parameter(self.prefix_ + "fixed_world_frame.frame.external", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_compensation.frame.id"):
                descriptor = ParameterDescriptor(description="frame which center of gravity (CoG) is defined in", read_only = False)
                parameter = rclpy.Parameter.Type.STRING
                self.node_.declare_parameter(self.prefix_ + "gravity_compensation.frame.id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_compensation.frame.external"):
                descriptor = ParameterDescriptor(description="specifies if the center of gravity frame is contained in the kinematics chain from the base to the tip", read_only = False)
                parameter = updated_params.gravity_compensation.frame.external
                self.node_.declare_parameter(self.prefix_ + "gravity_compensation.frame.external", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_compensation.CoG.pos"):
                descriptor = ParameterDescriptor(description="position of the center of gravity (CoG) in its frame", read_only = False)
                parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                self.node_.declare_parameter(self.prefix_ + "gravity_compensation.CoG.pos", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gravity_compensation.CoG.force"):
                descriptor = ParameterDescriptor(description="weight of the end effector, e.g mass * 9.81", read_only = False)
                parameter = updated_params.gravity_compensation.CoG.force
                self.node_.declare_parameter(self.prefix_ + "gravity_compensation.CoG.force", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "admittance.selected_axes"):
                descriptor = ParameterDescriptor(description="specifies if the axes x, y, z, rx, ry, and rz are enabled", read_only = False)
                parameter = rclpy.Parameter.Type.BOOL_ARRAY
                self.node_.declare_parameter(self.prefix_ + "admittance.selected_axes", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "admittance.mass"):
                descriptor = ParameterDescriptor(description="specifies mass values for x, y, z, rx, ry, and rz used in the admittance calculation", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = 1000000.0
                parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                self.node_.declare_parameter(self.prefix_ + "admittance.mass", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "admittance.damping_ratio"):
                descriptor = ParameterDescriptor(description="specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 10.0
                parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                self.node_.declare_parameter(self.prefix_ + "admittance.damping_ratio", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "admittance.stiffness"):
                descriptor = ParameterDescriptor(description="specifies stiffness values for x, y, z, rx, ry, and rz used in the admittance calculation", read_only = False)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.0001
                descriptor.floating_point_range[-1].to_value = 100000.0
                parameter = rclpy.Parameter.Type.DOUBLE_ARRAY
                self.node_.declare_parameter(self.prefix_ + "admittance.stiffness", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "enable_parameter_update_without_reactivation"):
                descriptor = ParameterDescriptor(description="if enabled, read_only parameters will be dynamically updated in the control loop", read_only = False)
                parameter = updated_params.enable_parameter_update_without_reactivation
                self.node_.declare_parameter(self.prefix_ + "enable_parameter_update_without_reactivation", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "use_feedforward_commanded_input"):
                descriptor = ParameterDescriptor(description="if enabled, the velocity commanded to the admittance controller is added to its calculated admittance velocity", read_only = False)
                parameter = updated_params.use_feedforward_commanded_input
                self.node_.declare_parameter(self.prefix_ + "use_feedforward_commanded_input", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "lt_eq_fifteen"):
                descriptor = ParameterDescriptor(description="should be a number less than or equal to 15", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = -2**31-1
                descriptor.integer_range[-1].to_value = 15
                parameter = updated_params.lt_eq_fifteen
                self.node_.declare_parameter(self.prefix_ + "lt_eq_fifteen", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "gt_fifteen"):
                descriptor = ParameterDescriptor(description="should be a number greater than 15", read_only = False)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 15
                descriptor.integer_range[-1].to_value = 2**31-1
                parameter = updated_params.gt_fifteen
                self.node_.declare_parameter(self.prefix_ + "gt_fifteen", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "one_number"):
                descriptor = ParameterDescriptor(description="", read_only = True)
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1024
                descriptor.integer_range[-1].to_value = 65535
                parameter = updated_params.one_number
                self.node_.declare_parameter(self.prefix_ + "one_number", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "three_numbers"):
                descriptor = ParameterDescriptor(description="", read_only = True)
                parameter = updated_params.three_numbers
                self.node_.declare_parameter(self.prefix_ + "three_numbers", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "three_numbers_of_five"):
                descriptor = ParameterDescriptor(description="", read_only = True)
                parameter = updated_params.three_numbers_of_five
                self.node_.declare_parameter(self.prefix_ + "three_numbers_of_five", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "hover_override"):
                descriptor = ParameterDescriptor(description="Override hover action:\n    0: Hover\n    1: Push\n    2: Pull\n    -1: Do not override", read_only = False)
                parameter = updated_params.hover_override
                self.node_.declare_parameter(self.prefix_ + "hover_override", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "angle_wraparound"):
                descriptor = ParameterDescriptor(description="For joints that wrap around (without end stop, ie. are continuous), where the shortest rotation to the target position is the desired motion. If true, the position error :math:`e = normalize(s_d - s)` is normalized between :math:`-\pi, \pi`. Otherwise  :math:`e = s_d - s` is used, with the desired position :math:`s_d` and the measured position :math:`s` from the state interface.", read_only = False)
                parameter = updated_params.angle_wraparound
                self.node_.declare_parameter(self.prefix_ + "angle_wraparound", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "open_loop_control"):
                descriptor = ParameterDescriptor(description="Use controller in open-loop control mode \n    \n     * The controller ignores the states provided by hardware interface but using last commands as states for starting the trajectory interpolation.\n     * It deactivates the feedback control, see the ``gains`` structure. \n    \n     This is useful if hardware states are not following commands, i.e., an offset between those (typical for hydraulic manipulators). \n    \n     If this flag is set, the controller tries to read the values from the command interfaces on activation. If they have real numeric values, those will be used instead of state interfaces. Therefore it is important set command interfaces to NaN (i.e., ``std::numeric_limits<double>::quiet_NaN()``) or state values when the hardware is started.\n    ", read_only = True)
                parameter = updated_params.open_loop_control
                self.node_.declare_parameter(self.prefix_ + "open_loop_control", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "scientific_notation_num")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.scientific_notation_num = param.value
            param = self.node_.get_parameter(self.prefix_ + "interpolation_mode")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.one_of(param, ["spline", "linear"])
            if validation_result:
                raise InvalidParameterValueException('interpolation_mode',param.value, 'Invalid value set during initialization for parameter interpolation_mode: ' + validation_result)
            validation_result = custom_validators.no_args_validator(param)
            if validation_result:
                raise InvalidParameterValueException('interpolation_mode',param.value, 'Invalid value set during initialization for parameter interpolation_mode: ' + validation_result)
            updated_params.interpolation_mode = param.value
            param = self.node_.get_parameter(self.prefix_ + "subset_selection")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.subset_of(param, ["A", "B", "C"])
            if validation_result:
                raise InvalidParameterValueException('subset_selection',param.value, 'Invalid value set during initialization for parameter subset_selection: ' + validation_result)
            updated_params.subset_selection = param.value
            param = self.node_.get_parameter(self.prefix_ + "joints")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.joints = param.value
            param = self.node_.get_parameter(self.prefix_ + "dof_names")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.dof_names = param.value
            param = self.node_.get_parameter(self.prefix_ + "pid.rate")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.pid.rate = param.value
            param = self.node_.get_parameter(self.prefix_ + "fixed_string")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_lt(param, 26)
            if validation_result:
                raise InvalidParameterValueException('fixed_string',param.value, 'Invalid value set during initialization for parameter fixed_string: ' + validation_result)
            updated_params.fixed_string = param.value
            param = self.node_.get_parameter(self.prefix_ + "fixed_array")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_lt(param, 11)
            if validation_result:
                raise InvalidParameterValueException('fixed_array',param.value, 'Invalid value set during initialization for parameter fixed_array: ' + validation_result)
            updated_params.fixed_array = param.value
            param = self.node_.get_parameter(self.prefix_ + "fixed_string_no_default")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_lt(param, 26)
            if validation_result:
                raise InvalidParameterValueException('fixed_string_no_default',param.value, 'Invalid value set during initialization for parameter fixed_string_no_default: ' + validation_result)
            updated_params.fixed_string_no_default = param.value
            param = self.node_.get_parameter(self.prefix_ + "command_interfaces")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.command_interfaces = param.value
            param = self.node_.get_parameter(self.prefix_ + "state_interfaces")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.state_interfaces = param.value
            param = self.node_.get_parameter(self.prefix_ + "chainable_command_interfaces")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.chainable_command_interfaces = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.plugin_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.plugin_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.plugin_package")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.plugin_package = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.base")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.base = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.tip")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.tip = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.alpha")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.alpha = param.value
            param = self.node_.get_parameter(self.prefix_ + "kinematics.group_name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.kinematics.group_name = param.value
            param = self.node_.get_parameter(self.prefix_ + "ft_sensor.name")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ft_sensor.name = param.value
            param = self.node_.get_parameter(self.prefix_ + "ft_sensor.frame.id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ft_sensor.frame.id = param.value
            param = self.node_.get_parameter(self.prefix_ + "ft_sensor.frame.external")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ft_sensor.frame.external = param.value
            param = self.node_.get_parameter(self.prefix_ + "ft_sensor.filter_coefficient")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.ft_sensor.filter_coefficient = param.value
            param = self.node_.get_parameter(self.prefix_ + "control.frame.id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.control.frame.id = param.value
            param = self.node_.get_parameter(self.prefix_ + "control.frame.external")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.control.frame.external = param.value
            param = self.node_.get_parameter(self.prefix_ + "fixed_world_frame.frame.id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.fixed_world_frame.frame.id = param.value
            param = self.node_.get_parameter(self.prefix_ + "fixed_world_frame.frame.external")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.fixed_world_frame.frame.external = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_compensation.frame.id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gravity_compensation.frame.id = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_compensation.frame.external")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gravity_compensation.frame.external = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_compensation.CoG.pos")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.fixed_size(param, 3)
            if validation_result:
                raise InvalidParameterValueException('gravity_compensation.CoG.pos',param.value, 'Invalid value set during initialization for parameter gravity_compensation.CoG.pos: ' + validation_result)
            updated_params.gravity_compensation.CoG.pos = param.value
            param = self.node_.get_parameter(self.prefix_ + "gravity_compensation.CoG.force")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gravity_compensation.CoG.force = param.value
            param = self.node_.get_parameter(self.prefix_ + "admittance.selected_axes")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.fixed_size(param, 6)
            if validation_result:
                raise InvalidParameterValueException('admittance.selected_axes',param.value, 'Invalid value set during initialization for parameter admittance.selected_axes: ' + validation_result)
            updated_params.admittance.selected_axes = param.value
            param = self.node_.get_parameter(self.prefix_ + "admittance.mass")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.fixed_size(param, 6)
            if validation_result:
                raise InvalidParameterValueException('admittance.mass',param.value, 'Invalid value set during initialization for parameter admittance.mass: ' + validation_result)
            validation_result = ParameterValidators.element_bounds(param, 0.0001, 1000000.0)
            if validation_result:
                raise InvalidParameterValueException('admittance.mass',param.value, 'Invalid value set during initialization for parameter admittance.mass: ' + validation_result)
            updated_params.admittance.mass = param.value
            param = self.node_.get_parameter(self.prefix_ + "admittance.damping_ratio")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.fixed_size(param, 6)
            if validation_result:
                raise InvalidParameterValueException('admittance.damping_ratio',param.value, 'Invalid value set during initialization for parameter admittance.damping_ratio: ' + validation_result)
            validation_result = custom_validators.validate_double_array_custom_func(param, 20.3, 5.0)
            if validation_result:
                raise InvalidParameterValueException('admittance.damping_ratio',param.value, 'Invalid value set during initialization for parameter admittance.damping_ratio: ' + validation_result)
            validation_result = ParameterValidators.element_bounds(param, 0.1, 10.0)
            if validation_result:
                raise InvalidParameterValueException('admittance.damping_ratio',param.value, 'Invalid value set during initialization for parameter admittance.damping_ratio: ' + validation_result)
            updated_params.admittance.damping_ratio = param.value
            param = self.node_.get_parameter(self.prefix_ + "admittance.stiffness")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.element_bounds(param, 0.0001, 100000.0)
            if validation_result:
                raise InvalidParameterValueException('admittance.stiffness',param.value, 'Invalid value set during initialization for parameter admittance.stiffness: ' + validation_result)
            updated_params.admittance.stiffness = param.value
            param = self.node_.get_parameter(self.prefix_ + "enable_parameter_update_without_reactivation")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.enable_parameter_update_without_reactivation = param.value
            param = self.node_.get_parameter(self.prefix_ + "use_feedforward_commanded_input")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.use_feedforward_commanded_input = param.value
            param = self.node_.get_parameter(self.prefix_ + "lt_eq_fifteen")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.lt_eq(param, 15)
            if validation_result:
                raise InvalidParameterValueException('lt_eq_fifteen',param.value, 'Invalid value set during initialization for parameter lt_eq_fifteen: ' + validation_result)
            updated_params.lt_eq_fifteen = param.value
            param = self.node_.get_parameter(self.prefix_ + "gt_fifteen")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.gt(param, 15)
            if validation_result:
                raise InvalidParameterValueException('gt_fifteen',param.value, 'Invalid value set during initialization for parameter gt_fifteen: ' + validation_result)
            updated_params.gt_fifteen = param.value
            param = self.node_.get_parameter(self.prefix_ + "one_number")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1024, 65535)
            if validation_result:
                raise InvalidParameterValueException('one_number',param.value, 'Invalid value set during initialization for parameter one_number: ' + validation_result)
            updated_params.one_number = param.value
            param = self.node_.get_parameter(self.prefix_ + "three_numbers")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.three_numbers = param.value
            param = self.node_.get_parameter(self.prefix_ + "three_numbers_of_five")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.size_lt(param, 6)
            if validation_result:
                raise InvalidParameterValueException('three_numbers_of_five',param.value, 'Invalid value set during initialization for parameter three_numbers_of_five: ' + validation_result)
            updated_params.three_numbers_of_five = param.value
            param = self.node_.get_parameter(self.prefix_ + "hover_override")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.one_of(param, [0, 1, 2, -1])
            if validation_result:
                raise InvalidParameterValueException('hover_override',param.value, 'Invalid value set during initialization for parameter hover_override: ' + validation_result)
            updated_params.hover_override = param.value
            param = self.node_.get_parameter(self.prefix_ + "angle_wraparound")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.angle_wraparound = param.value
            param = self.node_.get_parameter(self.prefix_ + "open_loop_control")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.open_loop_control = param.value


            # declare and set all dynamic parameters


            for value_1 in updated_params.joints:


                for value_2 in updated_params.dof_names:


                    updated_params.add_entry(value_1).add_entry(value_2)
                    entry = updated_params.get_entry(value_1).get_entry(value_2)
                    param_name = f"{self.prefix_}{value_1}.{value_2}.weight"
                    if not self.node_.has_parameter(self.prefix_ + param_name):
                        descriptor = ParameterDescriptor(description="map parameter without struct name", read_only = False)
                        descriptor.floating_point_range.append(FloatingPointRange())
                        descriptor.floating_point_range[-1].from_value = 0.0
                        descriptor.floating_point_range[-1].to_value = float('inf')
                        parameter = entry.weight
                        self.node_.declare_parameter(param_name, parameter, descriptor)
                    param = self.node_.get_parameter(param_name)
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                    validation_result = ParameterValidators.gt(param, 0.0)
                    if validation_result:
                        raise InvalidParameterValueException('__map_joints.__map_dof_names.weight',param.value, 'Invalid value set during initialization for parameter __map_joints.__map_dof_names.weight: ' + validation_result)
                    entry.weight = param.value


            for value_1 in updated_params.nested_dynamic.joints:


                for value_2 in updated_params.nested_dynamic.dof_names:


                    updated_params.nested_dynamic.add_entry(value_1).add_entry(value_2)
                    entry = updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2)
                    param_name = f"{self.prefix_}nested_dynamic.{value_1}.{value_2}.nested"
                    if not self.node_.has_parameter(self.prefix_ + param_name):
                        descriptor = ParameterDescriptor(description="test nested map params", read_only = False)
                        descriptor.floating_point_range.append(FloatingPointRange())
                        descriptor.floating_point_range[-1].from_value = 0.0001
                        descriptor.floating_point_range[-1].to_value = float('inf')
                        parameter = entry.nested
                        self.node_.declare_parameter(param_name, parameter, descriptor)
                    param = self.node_.get_parameter(param_name)
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                    validation_result = ParameterValidators.gt_eq(param, 0.0001)
                    if validation_result:
                        raise InvalidParameterValueException('nested_dynamic.__map_joints.__map_dof_names.nested',param.value, 'Invalid value set during initialization for parameter nested_dynamic.__map_joints.__map_dof_names.nested: ' + validation_result)
                    entry.nested = param.value


            for value_1 in updated_params.nested_dynamic.joints:


                for value_2 in updated_params.nested_dynamic.dof_names:


                    for value_3 in updated_params.nested_dynamic.joints:


                        for value_4 in updated_params.nested_dynamic.dof_names:


                            updated_params.nested_dynamic.add_entry(value_1).add_entry(value_2).add_entry(value_3).add_entry(value_4)
                            entry = updated_params.nested_dynamic.get_entry(value_1).get_entry(value_2).get_entry(value_3).get_entry(value_4)
                            param_name = f"{self.prefix_}nested_dynamic.{value_1}.{value_2}.{value_3}.{value_4}.nested_deep"
                            if not self.node_.has_parameter(self.prefix_ + param_name):
                                descriptor = ParameterDescriptor(description="test deep nested map params", read_only = False)
                                descriptor.floating_point_range.append(FloatingPointRange())
                                descriptor.floating_point_range[-1].from_value = 0.0001
                                descriptor.floating_point_range[-1].to_value = float('inf')
                                parameter = entry.nested_deep
                                self.node_.declare_parameter(param_name, parameter, descriptor)
                            param = self.node_.get_parameter(param_name)
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                            validation_result = ParameterValidators.gt_eq(param, 0.0001)
                            if validation_result:
                                raise InvalidParameterValueException('nested_dynamic.__map_joints.__map_dof_names.__map_joints.__map_dof_names.nested_deep',param.value, 'Invalid value set during initialization for parameter nested_dynamic.__map_joints.__map_dof_names.__map_joints.__map_dof_names.nested_deep: ' + validation_result)
                            entry.nested_deep = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.p"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="proportional gain term", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0.0001
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.p
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0.0001)
                if validation_result:
                    raise InvalidParameterValueException('pid.__map_joints.p',param.value, 'Invalid value set during initialization for parameter pid.__map_joints.p: ' + validation_result)
                entry.p = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.i"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="integral gain term", read_only = False)
                    parameter = entry.i
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.i = param.value


            for value_1 in updated_params.pid.joints:


                updated_params.pid.add_entry(value_1)
                entry = updated_params.pid.get_entry(value_1)
                param_name = f"{self.prefix_}pid.{value_1}.d"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="derivative gain term", read_only = False)
                    parameter = entry.d
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.d = param.value


            for value_1 in updated_params.gains.dof_names:


                updated_params.gains.add_entry(value_1)
                entry = updated_params.gains.get_entry(value_1)
                param_name = f"{self.prefix_}gains.{value_1}.k"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="general gain", read_only = False)
                    parameter = entry.k
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.k = param.value

            self.update_internal_params(updated_params)
