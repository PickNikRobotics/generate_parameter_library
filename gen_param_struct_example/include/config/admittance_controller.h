// this is auto-generated code 

#include <rclcpp/node.hpp>
#include <vector>
#include <string>


namespace admittance_controller_parameters {

  struct admittance_controller {
    // if true, prevent parameters from updating
    bool lock_params_ = false;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;

    admittance_controller(
        const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      declare_params(parameters_interface);
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters) {
        return this->update(parameters);
      };
      handle_ = parameters_interface->add_on_set_parameters_callback(update_param_cb);
    }

    struct params {
      std::vector<std::string> joints_ = {"UNDEFINED"};
      std::vector<std::string> command_interfaces_ = {"UNDEFINED"};
      std::vector<std::string> state_interfaces_ = {"UNDEFINED"};
      std::vector<std::string> chainable_command_interfaces_ = {"UNDEFINED"};
      struct kinematics {
        std::string plugin_name_ = "UNDEFINED";
        std::string base_ = "UNDEFINED";
        std::string tip_ = "UNDEFINED";
      } kinematics_;
      struct ft_sensor {
        std::string name_ = "UNDEFINED";
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
      } ft_sensor_;
      struct control {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
      } control_;
      struct fixed_world_frame {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
      } fixed_world_frame_;
      struct gravity_compensation {
        struct frame {
          std::string id_ = "UNDEFINED";
          bool external_ = false;
        } frame_;
        struct CoG {
          std::vector<double> pos_ = {std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN()};
          double force_ = std::numeric_limits<double>::quiet_NaN();
        } CoG_;
      } gravity_compensation_;
      struct admittance {
        std::vector<bool> selected_axes_ = {false, false, false, false, false, false};
        std::vector<double> mass_ = {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN(),
                                     std::numeric_limits<double>::quiet_NaN()};
        std::vector<double> damping_ratio_ = {std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN(),
                                              std::numeric_limits<double>::quiet_NaN()};
        std::vector<double> stiffness_ = {std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN(),
                                          std::numeric_limits<double>::quiet_NaN()};
      } admittance_;
      bool enable_parameter_update_without_reactivation_ = true;

    } params_;

    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !lock_params_;
      if (lock_params_) {
        result.reason = "The parameters can not be updated because they are currently locked.";
        return result;
      }

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "joints") {
          params_.joints_ = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces") {
          params_.command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces") {
          params_.state_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces") {
          params_.chainable_command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "kinematics.plugin_name") {
          params_.kinematics_.plugin_name_ = param.as_string();
        }
        if (param.get_name() == "kinematics.base") {
          params_.kinematics_.base_ = param.as_string();
        }
        if (param.get_name() == "kinematics.tip") {
          params_.kinematics_.tip_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.name") {
          params_.ft_sensor_.name_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.id") {
          params_.ft_sensor_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "ft_sensor.frame.external") {
          params_.ft_sensor_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "control.frame.id") {
          params_.control_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "control.frame.external") {
          params_.control_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "fixed_world_frame.frame.id") {
          params_.fixed_world_frame_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "fixed_world_frame.frame.external") {
          params_.fixed_world_frame_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.frame.id") {
          params_.gravity_compensation_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "gravity_compensation.frame.external") {
          params_.gravity_compensation_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "gravity_compensation.CoG.pos") {
          params_.gravity_compensation_.CoG_.pos_ = param.as_double_array();
        }
        if (param.get_name() == "gravity_compensation.CoG.force") {
          params_.gravity_compensation_.CoG_.force_ = param.as_double();
        }
        if (param.get_name() == "admittance.selected_axes") {
          params_.admittance_.selected_axes_ = param.as_bool_array();
        }
        if (param.get_name() == "admittance.mass") {
          params_.admittance_.mass_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.damping_ratio") {
          params_.admittance_.damping_ratio_ = param.as_double_array();
        }
        if (param.get_name() == "admittance.stiffness") {
          params_.admittance_.stiffness_ = param.as_double_array();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          params_.enable_parameter_update_without_reactivation_ = param.as_bool();
        }

      }
      return result;
    }

    void declare_params(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      if (!parameters_interface->has_parameter("joints")) {
        auto p_joints = rclcpp::ParameterValue(params_.joints_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which joints will be used by the controller";
        descriptor.read_only = true;
        parameters_interface->declare_parameter("joints", p_joints, descriptor);
      }
      params_.joints_ = parameters_interface->get_parameter("joints").as_string_array();
      if (!parameters_interface->has_parameter("command_interfaces")) {
        auto p_command_interfaces = rclcpp::ParameterValue(params_.command_interfaces_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which command interfaces to claim";
        descriptor.read_only = true;
        parameters_interface->declare_parameter("command_interfaces", p_command_interfaces, descriptor);
      }
      params_.command_interfaces_ = parameters_interface->get_parameter("command_interfaces").as_string_array();
      if (!parameters_interface->has_parameter("state_interfaces")) {
        auto p_state_interfaces = rclcpp::ParameterValue(params_.state_interfaces_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which state interfaces to claim";
        descriptor.read_only = true;
        parameters_interface->declare_parameter("state_interfaces", p_state_interfaces, descriptor);
      }
      params_.state_interfaces_ = parameters_interface->get_parameter("state_interfaces").as_string_array();
      if (!parameters_interface->has_parameter("chainable_command_interfaces")) {
        auto p_chainable_command_interfaces = rclcpp::ParameterValue(params_.chainable_command_interfaces_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which chainable interfaces to claim";
        descriptor.read_only = true;
        parameters_interface->declare_parameter("chainable_command_interfaces", p_chainable_command_interfaces,
                                                descriptor);
      }
      params_.chainable_command_interfaces_ = parameters_interface->get_parameter(
          "chainable_command_interfaces").as_string_array();
      if (!parameters_interface->has_parameter("kinematics.plugin_name")) {
        auto p_kinematics_plugin_name = rclcpp::ParameterValue(params_.kinematics_.plugin_name_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies which kinematics plugin to load";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("kinematics.plugin_name", p_kinematics_plugin_name, descriptor);
      }
      params_.kinematics_.plugin_name_ = parameters_interface->get_parameter("kinematics.plugin_name").as_string();
      if (!parameters_interface->has_parameter("kinematics.base")) {
        auto p_kinematics_base = rclcpp::ParameterValue(params_.kinematics_.base_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the base link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("kinematics.base", p_kinematics_base, descriptor);
      }
      params_.kinematics_.base_ = parameters_interface->get_parameter("kinematics.base").as_string();
      if (!parameters_interface->has_parameter("kinematics.tip")) {
        auto p_kinematics_tip = rclcpp::ParameterValue(params_.kinematics_.tip_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies the end effector link of the robot description used by the kinematics plugin";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("kinematics.tip", p_kinematics_tip, descriptor);
      }
      params_.kinematics_.tip_ = parameters_interface->get_parameter("kinematics.tip").as_string();
      if (!parameters_interface->has_parameter("ft_sensor.name")) {
        auto p_ft_sensor_name = rclcpp::ParameterValue(params_.ft_sensor_.name_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "name of the force torque sensor in the robot description";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("ft_sensor.name", p_ft_sensor_name, descriptor);
      }
      params_.ft_sensor_.name_ = parameters_interface->get_parameter("ft_sensor.name").as_string();
      if (!parameters_interface->has_parameter("ft_sensor.frame.id")) {
        auto p_ft_sensor_frame_id = rclcpp::ParameterValue(params_.ft_sensor_.frame_.id_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame of the force torque sensor";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("ft_sensor.frame.id", p_ft_sensor_frame_id, descriptor);
      }
      params_.ft_sensor_.frame_.id_ = parameters_interface->get_parameter("ft_sensor.frame.id").as_string();
      if (!parameters_interface->has_parameter("ft_sensor.frame.external")) {
        auto p_ft_sensor_frame_external = rclcpp::ParameterValue(params_.ft_sensor_.frame_.external_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the force torque sensor is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("ft_sensor.frame.external", p_ft_sensor_frame_external, descriptor);
      }
      params_.ft_sensor_.frame_.external_ = parameters_interface->get_parameter("ft_sensor.frame.external").as_bool();
      if (!parameters_interface->has_parameter("control.frame.id")) {
        auto p_control_frame_id = rclcpp::ParameterValue(params_.control_.frame_.id_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "control frame used for admittance control";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("control.frame.id", p_control_frame_id, descriptor);
      }
      params_.control_.frame_.id_ = parameters_interface->get_parameter("control.frame.id").as_string();
      if (!parameters_interface->has_parameter("control.frame.external")) {
        auto p_control_frame_external = rclcpp::ParameterValue(params_.control_.frame_.external_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the control frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("control.frame.external", p_control_frame_external, descriptor);
      }
      params_.control_.frame_.external_ = parameters_interface->get_parameter("control.frame.external").as_bool();
      if (!parameters_interface->has_parameter("fixed_world_frame.frame.id")) {
        auto p_fixed_world_frame_frame_id = rclcpp::ParameterValue(params_.fixed_world_frame_.frame_.id_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "world frame, gravity points down (neg. Z) in this frame";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("fixed_world_frame.frame.id", p_fixed_world_frame_frame_id, descriptor);
      }
      params_.fixed_world_frame_.frame_.id_ = parameters_interface->get_parameter(
          "fixed_world_frame.frame.id").as_string();
      if (!parameters_interface->has_parameter("fixed_world_frame.frame.external")) {
        auto p_fixed_world_frame_frame_external = rclcpp::ParameterValue(params_.fixed_world_frame_.frame_.external_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the world frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("fixed_world_frame.frame.external", p_fixed_world_frame_frame_external,
                                                descriptor);
      }
      params_.fixed_world_frame_.frame_.external_ = parameters_interface->get_parameter(
          "fixed_world_frame.frame.external").as_bool();
      if (!parameters_interface->has_parameter("gravity_compensation.frame.id")) {
        auto p_gravity_compensation_frame_id = rclcpp::ParameterValue(params_.gravity_compensation_.frame_.id_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "frame which center of gravity (CoG) is defined in";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("gravity_compensation.frame.id", p_gravity_compensation_frame_id,
                                                descriptor);
      }
      params_.gravity_compensation_.frame_.id_ = parameters_interface->get_parameter(
          "gravity_compensation.frame.id").as_string();
      if (!parameters_interface->has_parameter("gravity_compensation.frame.external")) {
        auto p_gravity_compensation_frame_external = rclcpp::ParameterValue(
            params_.gravity_compensation_.frame_.external_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the center of gravity frame is contained in the kinematics chain from the base to the tip";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("gravity_compensation.frame.external",
                                                p_gravity_compensation_frame_external, descriptor);
      }
      params_.gravity_compensation_.frame_.external_ = parameters_interface->get_parameter(
          "gravity_compensation.frame.external").as_bool();
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.pos")) {
        auto p_gravity_compensation_CoG_pos = rclcpp::ParameterValue(params_.gravity_compensation_.CoG_.pos_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "position of the center of gravity (CoG) in its frame";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = -std::numeric_limits<double>::infinity();
        range.to_value = std::numeric_limits<double>::infinity();
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        parameters_interface->declare_parameter("gravity_compensation.CoG.pos", p_gravity_compensation_CoG_pos,
                                                descriptor);
      }
      params_.gravity_compensation_.CoG_.pos_ = parameters_interface->get_parameter(
          "gravity_compensation.CoG.pos").as_double_array();
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.force")) {
        auto p_gravity_compensation_CoG_force = rclcpp::ParameterValue(params_.gravity_compensation_.CoG_.force_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "weight of the end effector, e.g mass * 9.81";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = -std::numeric_limits<double>::infinity();
        range.to_value = std::numeric_limits<double>::infinity();
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        parameters_interface->declare_parameter("gravity_compensation.CoG.force", p_gravity_compensation_CoG_force,
                                                descriptor);
      }
      params_.gravity_compensation_.CoG_.force_ = parameters_interface->get_parameter(
          "gravity_compensation.CoG.force").as_double();
      if (!parameters_interface->has_parameter("admittance.selected_axes")) {
        auto p_admittance_selected_axes = rclcpp::ParameterValue(params_.admittance_.selected_axes_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies if the axes x, y, z, rx, ry, and rz are enabled";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("admittance.selected_axes", p_admittance_selected_axes, descriptor);
      }
      params_.admittance_.selected_axes_ = parameters_interface->get_parameter(
          "admittance.selected_axes").as_bool_array();
      if (!parameters_interface->has_parameter("admittance.mass")) {
        auto p_admittance_mass = rclcpp::ParameterValue(params_.admittance_.mass_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies mass values for x, y, z, rx, ry, and rz used in the admittance calculation";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = -std::numeric_limits<double>::infinity();
        range.to_value = std::numeric_limits<double>::infinity();
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        parameters_interface->declare_parameter("admittance.mass", p_admittance_mass, descriptor);
      }
      params_.admittance_.mass_ = parameters_interface->get_parameter("admittance.mass").as_double_array();
      if (!parameters_interface->has_parameter("admittance.damping_ratio")) {
        auto p_admittance_damping_ratio = rclcpp::ParameterValue(params_.admittance_.damping_ratio_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies damping ratio values for x, y, z, rx, ry, and rz used in the admittance calculation. The values are calculated as damping can be used instead: zeta = D / (2 * sqrt( M * S ))";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = -std::numeric_limits<double>::infinity();
        range.to_value = std::numeric_limits<double>::infinity();
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        parameters_interface->declare_parameter("admittance.damping_ratio", p_admittance_damping_ratio, descriptor);
      }
      params_.admittance_.damping_ratio_ = parameters_interface->get_parameter(
          "admittance.damping_ratio").as_double_array();
      if (!parameters_interface->has_parameter("admittance.stiffness")) {
        auto p_admittance_stiffness = rclcpp::ParameterValue(params_.admittance_.stiffness_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "specifies stiffness values for x, y, z, rx, ry, and rz used in the admittance calculation";
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = -std::numeric_limits<double>::infinity();
        range.to_value = std::numeric_limits<double>::infinity();
        descriptor.floating_point_range.push_back(range);
        descriptor.read_only = false;
        parameters_interface->declare_parameter("admittance.stiffness", p_admittance_stiffness, descriptor);
      }
      params_.admittance_.stiffness_ = parameters_interface->get_parameter("admittance.stiffness").as_double_array();
      if (!parameters_interface->has_parameter("enable_parameter_update_without_reactivation")) {
        auto p_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
            params_.enable_parameter_update_without_reactivation_);
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.description = "if enabled, configurable parameters will be dynamically updated in the control loop";
        descriptor.read_only = false;
        parameters_interface->declare_parameter("enable_parameter_update_without_reactivation",
                                                p_enable_parameter_update_without_reactivation, descriptor);
      }
      params_.enable_parameter_update_without_reactivation_ = parameters_interface->get_parameter(
          "enable_parameter_update_without_reactivation").as_bool();

    }
  };

} // namespace admittance_controller_parameters
