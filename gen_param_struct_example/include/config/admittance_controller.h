// this is auto-generated code 

#include <rclcpp/node.hpp>
#include <vector>
#include <string>


namespace admittance_controller_parameters {

  struct admittance_controller {
    // if true, prevent parameters from updating
    bool lock_params_ = false;
    std::shared_ptr <rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;

    admittance_controller(
        const std::shared_ptr <rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      declare_params(parameters_interface);
      auto update_param_cb = [this](const std::vector <rclcpp::Parameter> &parameters) {
        return this->update(parameters);
      };
      handle_ = parameters_interface->add_on_set_parameters_callback(update_param_cb);
    }

    struct params {
      struct joints {
        std::string joints_ = "UNDEFINED";
      } joints_;
      struct command_interfaces {
        std::string command_interfaces_ = "UNDEFINED";
      } command_interfaces_;
      struct state_interfaces {
        std::string state_interfaces_ = "UNDEFINED";
      } state_interfaces_;
      struct chainable_command_interfaces {
        std::string chainable_command_interfaces_ = "UNDEFINED";
      } chainable_command_interfaces_;
      struct kinematics {
        struct plugin_name {
          std::string plugin_name_ = "UNDEFINED";
        } plugin_name_;
        struct base {
          std::string base_ = "UNDEFINED";
        } base_;
        struct tip {
          std::string tip_ = "UNDEFINED";
        } tip_;
      } kinematics_;
      struct ft_sensor {
        struct name {
          std::string name_ = "UNDEFINED";
        } name_;
        struct frame {
          struct id {
            std::string id_ = "UNDEFINED";
          } id_;
          struct external {
            bool external_ = false;
          } external_;
        } frame_;
      } ft_sensor_;
      struct control {
        struct frame {
          struct id {
            std::string id_ = "UNDEFINED";
          } id_;
          struct external {
            bool external_ = false;
          } external_;
        } frame_;
      } control_;
      struct fixed_world_frame {
        struct frame {
          struct id {
            std::string id_ = "UNDEFINED";
          } id_;
          struct external {
            bool external_ = false;
          } external_;
        } frame_;
      } fixed_world_frame_;
      struct gravity_compensation {
        struct frame {
          struct id {
            std::string id_ = "UNDEFINED";
          } id_;
          struct external {
            bool external_ = false;
          } external_;
        } frame_;
        struct CoG {
          struct pos {
            std::vector<double> pos_ = {nan, nan, nan};
          } pos_;
          struct force {
            double force_ = nan;
          } force_;
        } CoG_;
      } gravity_compensation_;
      struct admittance {
        struct selected_axes {
          std::vector<bool> selected_axes_ = {false, false, false, false, false, false};
        } selected_axes_;
        struct mass {
          std::vector<double> mass_ = {nan, nan, nan, nan, nan, nan};
        } mass_;
        struct damping_ratio {
          std::vector<double> damping_ratio_ = {nan, nan, nan, nan, nan};
        } damping_ratio_;
        struct stiffness {
          std::vector<double> stiffness_ = {nan, nan, nan, nan, nan, nan};
        } stiffness_;
      } admittance_;
      struct enable_parameter_update_without_reactivation {
        bool enable_parameter_update_without_reactivation_ = true;
      } enable_parameter_update_without_reactivation_;

    } params_;

    rcl_interfaces::msg::SetParametersResult update(const std::vector <rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !lock_params_;
      if (lock_params_) {
        result.reason = "The parameters can not be updated because they are currently locked.";
        return result;
      }

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "admittance_controller.joints") {
          params_.admittance_controller_.joints_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.command_interfaces") {
          params_.admittance_controller_.command_interfaces_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.state_interfaces") {
          params_.admittance_controller_.state_interfaces_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.chainable_command_interfaces") {
          params_.admittance_controller_.chainable_command_interfaces_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.kinematics.plugin_name") {
          params_.admittance_controller_.kinematics_.plugin_name_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.kinematics.base") {
          params_.admittance_controller_.kinematics_.base_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.kinematics.tip") {
          params_.admittance_controller_.kinematics_.tip_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.ft_sensor.name") {
          params_.admittance_controller_.ft_sensor_.name_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.ft_sensor.frame.id") {
          params_.admittance_controller_.ft_sensor_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.ft_sensor.frame.external") {
          params_.admittance_controller_.ft_sensor_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "admittance_controller.control.frame.id") {
          params_.admittance_controller_.control_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.control.frame.external") {
          params_.admittance_controller_.control_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "admittance_controller.fixed_world_frame.frame.id") {
          params_.admittance_controller_.fixed_world_frame_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.fixed_world_frame.frame.external") {
          params_.admittance_controller_.fixed_world_frame_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "admittance_controller.gravity_compensation.frame.id") {
          params_.admittance_controller_.gravity_compensation_.frame_.id_ = param.as_string();
        }
        if (param.get_name() == "admittance_controller.gravity_compensation.frame.external") {
          params_.admittance_controller_.gravity_compensation_.frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "admittance_controller.gravity_compensation.CoG.pos") {
          params_.admittance_controller_.gravity_compensation_.CoG_.pos_ = param.as_double_array();
        }
        if (param.get_name() == "admittance_controller.gravity_compensation.CoG.force") {
          params_.admittance_controller_.gravity_compensation_.CoG_.force_ = param.as_double();
        }
        if (param.get_name() == "admittance_controller.admittance.selected_axes") {
          params_.admittance_controller_.admittance_.selected_axes_ = param.as_bool_array();
        }
        if (param.get_name() == "admittance_controller.admittance.mass") {
          params_.admittance_controller_.admittance_.mass_ = param.as_double_array();
        }
        if (param.get_name() == "admittance_controller.admittance.damping_ratio") {
          params_.admittance_controller_.admittance_.damping_ratio_ = param.as_double_array();
        }
        if (param.get_name() == "admittance_controller.admittance.stiffness") {
          params_.admittance_controller_.admittance_.stiffness_ = param.as_double_array();
        }
        if (param.get_name() == "admittance_controller.enable_parameter_update_without_reactivation") {
          params_.admittance_controller_.enable_parameter_update_without_reactivation_ = param.as_bool();
        }

      }
      return result;
    }

    void
    declare_params(const std::shared_ptr <rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      if (!parameters_interface->has_parameter("admittance_controller.joints")) {
        auto p_admittance_controller_joints = rclcpp::ParameterValue(params_.admittance_controller_.joints_);
        parameters_interface->declare_parameter("admittance_controller.joints", p_admittance_controller_joints);
      } else {
        params_.admittance_controller_.joints_ = parameters_interface->get_parameter(
            "admittance_controller.joints").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.command_interfaces")) {
        auto p_admittance_controller_command_interfaces = rclcpp::ParameterValue(
            params_.admittance_controller_.command_interfaces_);
        parameters_interface->declare_parameter("admittance_controller.command_interfaces",
                                                p_admittance_controller_command_interfaces);
      } else {
        params_.admittance_controller_.command_interfaces_ = parameters_interface->get_parameter(
            "admittance_controller.command_interfaces").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.state_interfaces")) {
        auto p_admittance_controller_state_interfaces = rclcpp::ParameterValue(
            params_.admittance_controller_.state_interfaces_);
        parameters_interface->declare_parameter("admittance_controller.state_interfaces",
                                                p_admittance_controller_state_interfaces);
      } else {
        params_.admittance_controller_.state_interfaces_ = parameters_interface->get_parameter(
            "admittance_controller.state_interfaces").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.chainable_command_interfaces")) {
        auto p_admittance_controller_chainable_command_interfaces = rclcpp::ParameterValue(
            params_.admittance_controller_.chainable_command_interfaces_);
        parameters_interface->declare_parameter("admittance_controller.chainable_command_interfaces",
                                                p_admittance_controller_chainable_command_interfaces);
      } else {
        params_.admittance_controller_.chainable_command_interfaces_ = parameters_interface->get_parameter(
            "admittance_controller.chainable_command_interfaces").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.kinematics.plugin_name")) {
        auto p_admittance_controller_kinematics_plugin_name = rclcpp::ParameterValue(
            params_.admittance_controller_.kinematics_.plugin_name_);
        parameters_interface->declare_parameter("admittance_controller.kinematics.plugin_name",
                                                p_admittance_controller_kinematics_plugin_name);
      } else {
        params_.admittance_controller_.kinematics_.plugin_name_ = parameters_interface->get_parameter(
            "admittance_controller.kinematics.plugin_name").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.kinematics.base")) {
        auto p_admittance_controller_kinematics_base = rclcpp::ParameterValue(
            params_.admittance_controller_.kinematics_.base_);
        parameters_interface->declare_parameter("admittance_controller.kinematics.base",
                                                p_admittance_controller_kinematics_base);
      } else {
        params_.admittance_controller_.kinematics_.base_ = parameters_interface->get_parameter(
            "admittance_controller.kinematics.base").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.kinematics.tip")) {
        auto p_admittance_controller_kinematics_tip = rclcpp::ParameterValue(
            params_.admittance_controller_.kinematics_.tip_);
        parameters_interface->declare_parameter("admittance_controller.kinematics.tip",
                                                p_admittance_controller_kinematics_tip);
      } else {
        params_.admittance_controller_.kinematics_.tip_ = parameters_interface->get_parameter(
            "admittance_controller.kinematics.tip").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.ft_sensor.name")) {
        auto p_admittance_controller_ft_sensor_name = rclcpp::ParameterValue(
            params_.admittance_controller_.ft_sensor_.name_);
        parameters_interface->declare_parameter("admittance_controller.ft_sensor.name",
                                                p_admittance_controller_ft_sensor_name);
      } else {
        params_.admittance_controller_.ft_sensor_.name_ = parameters_interface->get_parameter(
            "admittance_controller.ft_sensor.name").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.ft_sensor.frame.id")) {
        auto p_admittance_controller_ft_sensor_frame_id = rclcpp::ParameterValue(
            params_.admittance_controller_.ft_sensor_.frame_.id_);
        parameters_interface->declare_parameter("admittance_controller.ft_sensor.frame.id",
                                                p_admittance_controller_ft_sensor_frame_id);
      } else {
        params_.admittance_controller_.ft_sensor_.frame_.id_ = parameters_interface->get_parameter(
            "admittance_controller.ft_sensor.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.ft_sensor.frame.external")) {
        auto p_admittance_controller_ft_sensor_frame_external = rclcpp::ParameterValue(
            params_.admittance_controller_.ft_sensor_.frame_.external_);
        parameters_interface->declare_parameter("admittance_controller.ft_sensor.frame.external",
                                                p_admittance_controller_ft_sensor_frame_external);
      } else {
        params_.admittance_controller_.ft_sensor_.frame_.external_ = parameters_interface->get_parameter(
            "admittance_controller.ft_sensor.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("admittance_controller.control.frame.id")) {
        auto p_admittance_controller_control_frame_id = rclcpp::ParameterValue(
            params_.admittance_controller_.control_.frame_.id_);
        parameters_interface->declare_parameter("admittance_controller.control.frame.id",
                                                p_admittance_controller_control_frame_id);
      } else {
        params_.admittance_controller_.control_.frame_.id_ = parameters_interface->get_parameter(
            "admittance_controller.control.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.control.frame.external")) {
        auto p_admittance_controller_control_frame_external = rclcpp::ParameterValue(
            params_.admittance_controller_.control_.frame_.external_);
        parameters_interface->declare_parameter("admittance_controller.control.frame.external",
                                                p_admittance_controller_control_frame_external);
      } else {
        params_.admittance_controller_.control_.frame_.external_ = parameters_interface->get_parameter(
            "admittance_controller.control.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("admittance_controller.fixed_world_frame.frame.id")) {
        auto p_admittance_controller_fixed_world_frame_frame_id = rclcpp::ParameterValue(
            params_.admittance_controller_.fixed_world_frame_.frame_.id_);
        parameters_interface->declare_parameter("admittance_controller.fixed_world_frame.frame.id",
                                                p_admittance_controller_fixed_world_frame_frame_id);
      } else {
        params_.admittance_controller_.fixed_world_frame_.frame_.id_ = parameters_interface->get_parameter(
            "admittance_controller.fixed_world_frame.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.fixed_world_frame.frame.external")) {
        auto p_admittance_controller_fixed_world_frame_frame_external = rclcpp::ParameterValue(
            params_.admittance_controller_.fixed_world_frame_.frame_.external_);
        parameters_interface->declare_parameter("admittance_controller.fixed_world_frame.frame.external",
                                                p_admittance_controller_fixed_world_frame_frame_external);
      } else {
        params_.admittance_controller_.fixed_world_frame_.frame_.external_ = parameters_interface->get_parameter(
            "admittance_controller.fixed_world_frame.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("admittance_controller.gravity_compensation.frame.id")) {
        auto p_admittance_controller_gravity_compensation_frame_id = rclcpp::ParameterValue(
            params_.admittance_controller_.gravity_compensation_.frame_.id_);
        parameters_interface->declare_parameter("admittance_controller.gravity_compensation.frame.id",
                                                p_admittance_controller_gravity_compensation_frame_id);
      } else {
        params_.admittance_controller_.gravity_compensation_.frame_.id_ = parameters_interface->get_parameter(
            "admittance_controller.gravity_compensation.frame.id").as_string();
      }
      if (!parameters_interface->has_parameter("admittance_controller.gravity_compensation.frame.external")) {
        auto p_admittance_controller_gravity_compensation_frame_external = rclcpp::ParameterValue(
            params_.admittance_controller_.gravity_compensation_.frame_.external_);
        parameters_interface->declare_parameter("admittance_controller.gravity_compensation.frame.external",
                                                p_admittance_controller_gravity_compensation_frame_external);
      } else {
        params_.admittance_controller_.gravity_compensation_.frame_.external_ = parameters_interface->get_parameter(
            "admittance_controller.gravity_compensation.frame.external").as_bool();
      }
      if (!parameters_interface->has_parameter("admittance_controller.gravity_compensation.CoG.pos")) {
        auto p_admittance_controller_gravity_compensation_CoG_pos = rclcpp::ParameterValue(
            params_.admittance_controller_.gravity_compensation_.CoG_.pos_);
        parameters_interface->declare_parameter("admittance_controller.gravity_compensation.CoG.pos",
                                                p_admittance_controller_gravity_compensation_CoG_pos);
      } else {
        params_.admittance_controller_.gravity_compensation_.CoG_.pos_ = parameters_interface->get_parameter(
            "admittance_controller.gravity_compensation.CoG.pos").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance_controller.gravity_compensation.CoG.force")) {
        auto p_admittance_controller_gravity_compensation_CoG_force = rclcpp::ParameterValue(
            params_.admittance_controller_.gravity_compensation_.CoG_.force_);
        parameters_interface->declare_parameter("admittance_controller.gravity_compensation.CoG.force",
                                                p_admittance_controller_gravity_compensation_CoG_force);
      } else {
        params_.admittance_controller_.gravity_compensation_.CoG_.force_ = parameters_interface->get_parameter(
            "admittance_controller.gravity_compensation.CoG.force").as_double();
      }
      if (!parameters_interface->has_parameter("admittance_controller.admittance.selected_axes")) {
        auto p_admittance_controller_admittance_selected_axes = rclcpp::ParameterValue(
            params_.admittance_controller_.admittance_.selected_axes_);
        parameters_interface->declare_parameter("admittance_controller.admittance.selected_axes",
                                                p_admittance_controller_admittance_selected_axes);
      } else {
        params_.admittance_controller_.admittance_.selected_axes_ = parameters_interface->get_parameter(
            "admittance_controller.admittance.selected_axes").as_bool_array();
      }
      if (!parameters_interface->has_parameter("admittance_controller.admittance.mass")) {
        auto p_admittance_controller_admittance_mass = rclcpp::ParameterValue(
            params_.admittance_controller_.admittance_.mass_);
        parameters_interface->declare_parameter("admittance_controller.admittance.mass",
                                                p_admittance_controller_admittance_mass);
      } else {
        params_.admittance_controller_.admittance_.mass_ = parameters_interface->get_parameter(
            "admittance_controller.admittance.mass").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance_controller.admittance.damping_ratio")) {
        auto p_admittance_controller_admittance_damping_ratio = rclcpp::ParameterValue(
            params_.admittance_controller_.admittance_.damping_ratio_);
        parameters_interface->declare_parameter("admittance_controller.admittance.damping_ratio",
                                                p_admittance_controller_admittance_damping_ratio);
      } else {
        params_.admittance_controller_.admittance_.damping_ratio_ = parameters_interface->get_parameter(
            "admittance_controller.admittance.damping_ratio").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance_controller.admittance.stiffness")) {
        auto p_admittance_controller_admittance_stiffness = rclcpp::ParameterValue(
            params_.admittance_controller_.admittance_.stiffness_);
        parameters_interface->declare_parameter("admittance_controller.admittance.stiffness",
                                                p_admittance_controller_admittance_stiffness);
      } else {
        params_.admittance_controller_.admittance_.stiffness_ = parameters_interface->get_parameter(
            "admittance_controller.admittance.stiffness").as_double_array();
      }
      if (!parameters_interface->has_parameter("admittance_controller.enable_parameter_update_without_reactivation")) {
        auto p_admittance_controller_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
            params_.admittance_controller_.enable_parameter_update_without_reactivation_);
        parameters_interface->declare_parameter("admittance_controller.enable_parameter_update_without_reactivation",
                                                p_admittance_controller_enable_parameter_update_without_reactivation);
      } else {
        params_.admittance_controller_.enable_parameter_update_without_reactivation_ = parameters_interface->get_parameter(
            "admittance_controller.enable_parameter_update_without_reactivation").as_bool();
      }

    }
  };

} // namespace admittance_controller_parameters
