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

    std::vector <std::string> joints_ = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                         "wrist_2_joint", "wrist_3_joint"};
    std::vector <std::string> command_interfaces_ = {"position"};
    std::vector <std::string> state_interfaces_ = {"position", "velocity"};
    std::vector <std::string> chainable_command_interfaces_ = {"position", "velocity"};
    struct kinematics {
      std::string plugin_name_ = "kdl_plugin/KDLKinematics";
      std::string base_ = "base_link";
      std::string tip_ = "ee_link";
      std::string group_name_ = "ur5e_manipulator";
    } kinematics_;
    struct ft_sensor {
      std::string name_ = "tcp_fts_sensor";
      std::string frame_id_ = "ee_link";
      bool frame_external_ = false;
    } ft_sensor_;
    struct control {
      bool open_loop_control_ = false;
      std::string frame_id_ = "ee_link";
      bool frame_external_ = false;
    } control_;
    struct fixed_world_frame {
      std::string id_ = "base_link";
      bool external_ = false;
    } fixed_world_frame_;
    struct gravity_compensation {
      std::string frame_id_ = "ee_link";
      bool external_ = false;
      struct CoG {
        double x_ = 0.1;
        double y_ = 0.0;
        double z_ = 0.0;
        double force_ = 23.0;
      } CoG_;
    } gravity_compensation_;
    struct admittance {
      struct selected_axes {
        bool x_ = true;
        bool y_ = true;
        bool z_ = true;
        bool rx_ = true;
        bool ry_ = true;
        bool rz_ = true;
      } selected_axes_;
      struct mass {
        double x_ = 3.0;
        double y_ = 3.0;
        double z_ = 3.0;
        double rx_ = 0.05;
        double ry_ = 0.05;
        double rz_ = 0.05;
      } mass_;
      struct damping_ratio {
        double x_ = 2.828427;
        double y_ = 2.828427;
        double z_ = 2.828427;
        double rx_ = 2.23607;
        double ry_ = 2.23607;
        double rz_ = 2.23607;
      } damping_ratio_;
      struct stiffness {
        double x_ = 50.0;
        double y_ = 50.0;
        double z_ = 50.0;
        double rx_ = 1.0;
        double ry_ = 1.0;
        double rz_ = 1.0;
      } stiffness_;
    } admittance_;
    bool enable_parameter_update_without_reactivation_ = false;
    std::string joint_limiter_type_ = "joint_limits/SimpleJointLimiter";
    double state_publish_rate_ = 200.0;


    rcl_interfaces::msg::SetParametersResult update(const std::vector <rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !lock_params_;
      if (lock_params_) {
        result.reason = "The parameters can not be updated because they are currently locked.";
        return result;
      }

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "joints") {
          joints_ = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces") {
          command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces") {
          state_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces") {
          chainable_command_interfaces_ = param.as_string_array();
        }
        if (param.get_name() == "plugin_name") {
          kinematics_.plugin_name_ = param.as_string();
        }
        if (param.get_name() == "base") {
          kinematics_.base_ = param.as_string();
        }
        if (param.get_name() == "tip") {
          kinematics_.tip_ = param.as_string();
        }
        if (param.get_name() == "group_name") {
          kinematics_.group_name_ = param.as_string();
        }
        if (param.get_name() == "name") {
          ft_sensor_.name_ = param.as_string();
        }
        if (param.get_name() == "frame_id") {
          ft_sensor_.frame_id_ = param.as_string();
        }
        if (param.get_name() == "frame_external") {
          ft_sensor_.frame_external_ = param.as_bool();
        }
        if (param.get_name() == "open_loop_control") {
          control_.open_loop_control_ = param.as_bool();
        }
        if (param.get_name() == "frame_id") {
          control_.frame_id_ = param.as_string();
        }
        if (param.get_name() == "frame_external") {
          control_.frame_external_ = param.as_bool();
        }
        if (param.get_name() == "id") {
          fixed_world_frame_.id_ = param.as_string();
        }
        if (param.get_name() == "external") {
          fixed_world_frame_.external_ = param.as_bool();
        }
        if (param.get_name() == "frame_id") {
          gravity_compensation_.frame_id_ = param.as_string();
        }
        if (param.get_name() == "external") {
          gravity_compensation_.external_ = param.as_bool();
        }
        if (param.get_name() == "x") {
          gravity_compensation_.CoG_.x_ = param.as_double();
        }
        if (param.get_name() == "y") {
          gravity_compensation_.CoG_.y_ = param.as_double();
        }
        if (param.get_name() == "z") {
          gravity_compensation_.CoG_.z_ = param.as_double();
        }
        if (param.get_name() == "force") {
          gravity_compensation_.CoG_.force_ = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.selected_axes_.x_ = param.as_bool();
        }
        if (param.get_name() == "y") {
          admittance_.selected_axes_.y_ = param.as_bool();
        }
        if (param.get_name() == "z") {
          admittance_.selected_axes_.z_ = param.as_bool();
        }
        if (param.get_name() == "rx") {
          admittance_.selected_axes_.rx_ = param.as_bool();
        }
        if (param.get_name() == "ry") {
          admittance_.selected_axes_.ry_ = param.as_bool();
        }
        if (param.get_name() == "rz") {
          admittance_.selected_axes_.rz_ = param.as_bool();
        }
        if (param.get_name() == "x") {
          admittance_.mass_.x_ = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.mass_.y_ = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.mass_.z_ = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.mass_.rx_ = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.mass_.ry_ = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.mass_.rz_ = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.damping_ratio_.x_ = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.damping_ratio_.y_ = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.damping_ratio_.z_ = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.damping_ratio_.rx_ = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.damping_ratio_.ry_ = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.damping_ratio_.rz_ = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.stiffness_.x_ = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.stiffness_.y_ = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.stiffness_.z_ = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.stiffness_.rx_ = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.stiffness_.ry_ = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.stiffness_.rz_ = param.as_double();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          enable_parameter_update_without_reactivation_ = param.as_bool();
        }
        if (param.get_name() == "joint_limiter_type") {
          joint_limiter_type_ = param.as_string();
        }
        if (param.get_name() == "state_publish_rate") {
          state_publish_rate_ = param.as_double();
        }

      }
      return result;
    }

    void
    declare_params(const std::shared_ptr <rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      if (!parameters_interface->has_parameter("joints")) {
        auto p_joints = rclcpp::ParameterValue(joints_);
        parameters_interface->declare_parameter("joints", p_joints);
      }
      if (!parameters_interface->has_parameter("command_interfaces")) {
        auto p_command_interfaces = rclcpp::ParameterValue(command_interfaces_);
        parameters_interface->declare_parameter("command_interfaces", p_command_interfaces);
      }
      if (!parameters_interface->has_parameter("state_interfaces")) {
        auto p_state_interfaces = rclcpp::ParameterValue(state_interfaces_);
        parameters_interface->declare_parameter("state_interfaces", p_state_interfaces);
      }
      if (!parameters_interface->has_parameter("chainable_command_interfaces")) {
        auto p_chainable_command_interfaces = rclcpp::ParameterValue(chainable_command_interfaces_);
        parameters_interface->declare_parameter("chainable_command_interfaces", p_chainable_command_interfaces);
      }
      if (!parameters_interface->has_parameter("kinematics.plugin_name")) {
        auto p_kinematics_plugin_name = rclcpp::ParameterValue(kinematics_.plugin_name_);
        parameters_interface->declare_parameter("kinematics.plugin_name", p_kinematics_plugin_name);
      }
      if (!parameters_interface->has_parameter("kinematics.base")) {
        auto p_kinematics_base = rclcpp::ParameterValue(kinematics_.base_);
        parameters_interface->declare_parameter("kinematics.base", p_kinematics_base);
      }
      if (!parameters_interface->has_parameter("kinematics.tip")) {
        auto p_kinematics_tip = rclcpp::ParameterValue(kinematics_.tip_);
        parameters_interface->declare_parameter("kinematics.tip", p_kinematics_tip);
      }
      if (!parameters_interface->has_parameter("kinematics.group_name")) {
        auto p_kinematics_group_name = rclcpp::ParameterValue(kinematics_.group_name_);
        parameters_interface->declare_parameter("kinematics.group_name", p_kinematics_group_name);
      }
      if (!parameters_interface->has_parameter("ft_sensor.name")) {
        auto p_ft_sensor_name = rclcpp::ParameterValue(ft_sensor_.name_);
        parameters_interface->declare_parameter("ft_sensor.name", p_ft_sensor_name);
      }
      if (!parameters_interface->has_parameter("ft_sensor.frame_id")) {
        auto p_ft_sensor_frame_id = rclcpp::ParameterValue(ft_sensor_.frame_id_);
        parameters_interface->declare_parameter("ft_sensor.frame_id", p_ft_sensor_frame_id);
      }
      if (!parameters_interface->has_parameter("ft_sensor.frame_external")) {
        auto p_ft_sensor_frame_external = rclcpp::ParameterValue(ft_sensor_.frame_external_);
        parameters_interface->declare_parameter("ft_sensor.frame_external", p_ft_sensor_frame_external);
      }
      if (!parameters_interface->has_parameter("control.open_loop_control")) {
        auto p_control_open_loop_control = rclcpp::ParameterValue(control_.open_loop_control_);
        parameters_interface->declare_parameter("control.open_loop_control", p_control_open_loop_control);
      }
      if (!parameters_interface->has_parameter("control.frame_id")) {
        auto p_control_frame_id = rclcpp::ParameterValue(control_.frame_id_);
        parameters_interface->declare_parameter("control.frame_id", p_control_frame_id);
      }
      if (!parameters_interface->has_parameter("control.frame_external")) {
        auto p_control_frame_external = rclcpp::ParameterValue(control_.frame_external_);
        parameters_interface->declare_parameter("control.frame_external", p_control_frame_external);
      }
      if (!parameters_interface->has_parameter("fixed_world_frame.id")) {
        auto p_fixed_world_frame_id = rclcpp::ParameterValue(fixed_world_frame_.id_);
        parameters_interface->declare_parameter("fixed_world_frame.id", p_fixed_world_frame_id);
      }
      if (!parameters_interface->has_parameter("fixed_world_frame.external")) {
        auto p_fixed_world_frame_external = rclcpp::ParameterValue(fixed_world_frame_.external_);
        parameters_interface->declare_parameter("fixed_world_frame.external", p_fixed_world_frame_external);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.frame_id")) {
        auto p_gravity_compensation_frame_id = rclcpp::ParameterValue(gravity_compensation_.frame_id_);
        parameters_interface->declare_parameter("gravity_compensation.frame_id", p_gravity_compensation_frame_id);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.external")) {
        auto p_gravity_compensation_external = rclcpp::ParameterValue(gravity_compensation_.external_);
        parameters_interface->declare_parameter("gravity_compensation.external", p_gravity_compensation_external);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.x")) {
        auto p_gravity_compensation_CoG_x = rclcpp::ParameterValue(gravity_compensation_.CoG_.x_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.x", p_gravity_compensation_CoG_x);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.y")) {
        auto p_gravity_compensation_CoG_y = rclcpp::ParameterValue(gravity_compensation_.CoG_.y_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.y", p_gravity_compensation_CoG_y);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.z")) {
        auto p_gravity_compensation_CoG_z = rclcpp::ParameterValue(gravity_compensation_.CoG_.z_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.z", p_gravity_compensation_CoG_z);
      }
      if (!parameters_interface->has_parameter("gravity_compensation.CoG.force")) {
        auto p_gravity_compensation_CoG_force = rclcpp::ParameterValue(gravity_compensation_.CoG_.force_);
        parameters_interface->declare_parameter("gravity_compensation.CoG.force", p_gravity_compensation_CoG_force);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.x")) {
        auto p_admittance_selected_axes_x = rclcpp::ParameterValue(admittance_.selected_axes_.x_);
        parameters_interface->declare_parameter("admittance.selected_axes.x", p_admittance_selected_axes_x);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.y")) {
        auto p_admittance_selected_axes_y = rclcpp::ParameterValue(admittance_.selected_axes_.y_);
        parameters_interface->declare_parameter("admittance.selected_axes.y", p_admittance_selected_axes_y);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.z")) {
        auto p_admittance_selected_axes_z = rclcpp::ParameterValue(admittance_.selected_axes_.z_);
        parameters_interface->declare_parameter("admittance.selected_axes.z", p_admittance_selected_axes_z);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.rx")) {
        auto p_admittance_selected_axes_rx = rclcpp::ParameterValue(admittance_.selected_axes_.rx_);
        parameters_interface->declare_parameter("admittance.selected_axes.rx", p_admittance_selected_axes_rx);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.ry")) {
        auto p_admittance_selected_axes_ry = rclcpp::ParameterValue(admittance_.selected_axes_.ry_);
        parameters_interface->declare_parameter("admittance.selected_axes.ry", p_admittance_selected_axes_ry);
      }
      if (!parameters_interface->has_parameter("admittance.selected_axes.rz")) {
        auto p_admittance_selected_axes_rz = rclcpp::ParameterValue(admittance_.selected_axes_.rz_);
        parameters_interface->declare_parameter("admittance.selected_axes.rz", p_admittance_selected_axes_rz);
      }
      if (!parameters_interface->has_parameter("admittance.mass.x")) {
        auto p_admittance_mass_x = rclcpp::ParameterValue(admittance_.mass_.x_);
        parameters_interface->declare_parameter("admittance.mass.x", p_admittance_mass_x);
      }
      if (!parameters_interface->has_parameter("admittance.mass.y")) {
        auto p_admittance_mass_y = rclcpp::ParameterValue(admittance_.mass_.y_);
        parameters_interface->declare_parameter("admittance.mass.y", p_admittance_mass_y);
      }
      if (!parameters_interface->has_parameter("admittance.mass.z")) {
        auto p_admittance_mass_z = rclcpp::ParameterValue(admittance_.mass_.z_);
        parameters_interface->declare_parameter("admittance.mass.z", p_admittance_mass_z);
      }
      if (!parameters_interface->has_parameter("admittance.mass.rx")) {
        auto p_admittance_mass_rx = rclcpp::ParameterValue(admittance_.mass_.rx_);
        parameters_interface->declare_parameter("admittance.mass.rx", p_admittance_mass_rx);
      }
      if (!parameters_interface->has_parameter("admittance.mass.ry")) {
        auto p_admittance_mass_ry = rclcpp::ParameterValue(admittance_.mass_.ry_);
        parameters_interface->declare_parameter("admittance.mass.ry", p_admittance_mass_ry);
      }
      if (!parameters_interface->has_parameter("admittance.mass.rz")) {
        auto p_admittance_mass_rz = rclcpp::ParameterValue(admittance_.mass_.rz_);
        parameters_interface->declare_parameter("admittance.mass.rz", p_admittance_mass_rz);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.x")) {
        auto p_admittance_damping_ratio_x = rclcpp::ParameterValue(admittance_.damping_ratio_.x_);
        parameters_interface->declare_parameter("admittance.damping_ratio.x", p_admittance_damping_ratio_x);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.y")) {
        auto p_admittance_damping_ratio_y = rclcpp::ParameterValue(admittance_.damping_ratio_.y_);
        parameters_interface->declare_parameter("admittance.damping_ratio.y", p_admittance_damping_ratio_y);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.z")) {
        auto p_admittance_damping_ratio_z = rclcpp::ParameterValue(admittance_.damping_ratio_.z_);
        parameters_interface->declare_parameter("admittance.damping_ratio.z", p_admittance_damping_ratio_z);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.rx")) {
        auto p_admittance_damping_ratio_rx = rclcpp::ParameterValue(admittance_.damping_ratio_.rx_);
        parameters_interface->declare_parameter("admittance.damping_ratio.rx", p_admittance_damping_ratio_rx);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.ry")) {
        auto p_admittance_damping_ratio_ry = rclcpp::ParameterValue(admittance_.damping_ratio_.ry_);
        parameters_interface->declare_parameter("admittance.damping_ratio.ry", p_admittance_damping_ratio_ry);
      }
      if (!parameters_interface->has_parameter("admittance.damping_ratio.rz")) {
        auto p_admittance_damping_ratio_rz = rclcpp::ParameterValue(admittance_.damping_ratio_.rz_);
        parameters_interface->declare_parameter("admittance.damping_ratio.rz", p_admittance_damping_ratio_rz);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.x")) {
        auto p_admittance_stiffness_x = rclcpp::ParameterValue(admittance_.stiffness_.x_);
        parameters_interface->declare_parameter("admittance.stiffness.x", p_admittance_stiffness_x);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.y")) {
        auto p_admittance_stiffness_y = rclcpp::ParameterValue(admittance_.stiffness_.y_);
        parameters_interface->declare_parameter("admittance.stiffness.y", p_admittance_stiffness_y);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.z")) {
        auto p_admittance_stiffness_z = rclcpp::ParameterValue(admittance_.stiffness_.z_);
        parameters_interface->declare_parameter("admittance.stiffness.z", p_admittance_stiffness_z);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.rx")) {
        auto p_admittance_stiffness_rx = rclcpp::ParameterValue(admittance_.stiffness_.rx_);
        parameters_interface->declare_parameter("admittance.stiffness.rx", p_admittance_stiffness_rx);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.ry")) {
        auto p_admittance_stiffness_ry = rclcpp::ParameterValue(admittance_.stiffness_.ry_);
        parameters_interface->declare_parameter("admittance.stiffness.ry", p_admittance_stiffness_ry);
      }
      if (!parameters_interface->has_parameter("admittance.stiffness.rz")) {
        auto p_admittance_stiffness_rz = rclcpp::ParameterValue(admittance_.stiffness_.rz_);
        parameters_interface->declare_parameter("admittance.stiffness.rz", p_admittance_stiffness_rz);
      }
      if (!parameters_interface->has_parameter("enable_parameter_update_without_reactivation")) {
        auto p_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
            enable_parameter_update_without_reactivation_);
        parameters_interface->declare_parameter("enable_parameter_update_without_reactivation",
                                                p_enable_parameter_update_without_reactivation);
      }
      if (!parameters_interface->has_parameter("joint_limiter_type")) {
        auto p_joint_limiter_type = rclcpp::ParameterValue(joint_limiter_type_);
        parameters_interface->declare_parameter("joint_limiter_type", p_joint_limiter_type);
      }
      if (!parameters_interface->has_parameter("state_publish_rate")) {
        auto p_state_publish_rate = rclcpp::ParameterValue(state_publish_rate_);
        parameters_interface->declare_parameter("state_publish_rate", p_state_publish_rate);
      }

    }
  };

} // namespace admittance_controller_parameters
