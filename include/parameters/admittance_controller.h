// this is auto-generated code 

#include <rclcpp/node.hpp>
#include <vector>
#include <string>


namespace admittance_controller_parameters {

  struct admittance_controller {
    // if true, prevent parameters from updating
    bool lock_params = false;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> handle_;

    admittance_controller(
        const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      declare_params(parameters_interface);
      auto update_param_cb = [this](const std::vector<rclcpp::Parameter> &parameters) {
        return this->update(parameters);
      };
      handle_ = parameters_interface->add_on_set_parameters_callback(update_param_cb);
    }

    std::vector<std::string> joints = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                       "wrist_2_joint", "wrist_3_joint"};
    std::vector<std::string> command_interfaces = {"position"};
    std::vector<std::string> state_interfaces = {"position", "velocity"};
    std::vector<std::string> chainable_command_interfaces = {"position", "velocity"};
    struct kinematics {
      std::string plugin_name = "kdl_plugin/KDLKinematics";
      std::string base = "base_link";
      std::string tip = "ee_link";
      std::string group_name = "ur5e_manipulator";
    } kinematics_;
    struct ft_sensor {
      std::string name = "tcp_fts_sensor";
      std::string frame_id = "ee_link";
      bool frame_external = false;
    } ft_sensor_;
    struct control {
      bool open_loop_control = false;
      std::string frame_id = "ee_link";
      bool frame_external = false;
    } control_;
    struct fixed_world_frame {
      std::string id = "base_link";
      bool external = false;
    } fixed_world_frame_;
    struct gravity_compensation {
      std::string frame_id = "ee_link";
      bool external = false;
      struct CoG {
        double x = 0.1;
        double y = 0.0;
        double z = 0.0;
        double force = 23.0;
      } CoG_;
    } gravity_compensation_;
    struct admittance {
      struct selected_axes {
        bool x = true;
        bool y = true;
        bool z = true;
        bool rx = true;
        bool ry = true;
        bool rz = true;
      } selected_axes_;
      struct mass {
        double x = 3.0;
        double y = 3.0;
        double z = 3.0;
        double rx = 0.05;
        double ry = 0.05;
        double rz = 0.05;
      } mass_;
      struct damping_ratio {
        double x = 2.828427;
        double y = 2.828427;
        double z = 2.828427;
        double rx = 2.23607;
        double ry = 2.23607;
        double rz = 2.23607;
      } damping_ratio_;
      struct stiffness {
        double x = 50.0;
        double y = 50.0;
        double z = 50.0;
        double rx = 1.0;
        double ry = 1.0;
        double rz = 1.0;
      } stiffness_;
    } admittance_;
    bool enable_parameter_update_without_reactivation = false;
    std::string joint_limiter_type = "joint_limits/SimpleJointLimiter";
    double state_publish_rate = 200.0;


    rcl_interfaces::msg::SetParametersResult update(const std::vector<rclcpp::Parameter> &parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = !lock_params;
      if (lock_params) {
        result.reason = "The parameters can not be updated because they are currently locked.";
        return result;
      }

      result.reason = "success";
      for (const auto &param: parameters) {
        if (param.get_name() == "joints") {
          joints = param.as_string_array();
        }
        if (param.get_name() == "command_interfaces") {
          command_interfaces = param.as_string_array();
        }
        if (param.get_name() == "state_interfaces") {
          state_interfaces = param.as_string_array();
        }
        if (param.get_name() == "chainable_command_interfaces") {
          chainable_command_interfaces = param.as_string_array();
        }
        if (param.get_name() == "plugin_name") {
          kinematics_.plugin_name = param.as_string();
        }
        if (param.get_name() == "base") {
          kinematics_.base = param.as_string();
        }
        if (param.get_name() == "tip") {
          kinematics_.tip = param.as_string();
        }
        if (param.get_name() == "group_name") {
          kinematics_.group_name = param.as_string();
        }
        if (param.get_name() == "name") {
          ft_sensor_.name = param.as_string();
        }
        if (param.get_name() == "frame_id") {
          ft_sensor_.frame_id = param.as_string();
        }
        if (param.get_name() == "frame_external") {
          ft_sensor_.frame_external = param.as_bool();
        }
        if (param.get_name() == "open_loop_control") {
          control_.open_loop_control = param.as_bool();
        }
        if (param.get_name() == "frame_id") {
          control_.frame_id = param.as_string();
        }
        if (param.get_name() == "frame_external") {
          control_.frame_external = param.as_bool();
        }
        if (param.get_name() == "id") {
          fixed_world_frame_.id = param.as_string();
        }
        if (param.get_name() == "external") {
          fixed_world_frame_.external = param.as_bool();
        }
        if (param.get_name() == "frame_id") {
          gravity_compensation_.frame_id = param.as_string();
        }
        if (param.get_name() == "external") {
          gravity_compensation_.external = param.as_bool();
        }
        if (param.get_name() == "x") {
          gravity_compensation_.CoG_.x = param.as_double();
        }
        if (param.get_name() == "y") {
          gravity_compensation_.CoG_.y = param.as_double();
        }
        if (param.get_name() == "z") {
          gravity_compensation_.CoG_.z = param.as_double();
        }
        if (param.get_name() == "force") {
          gravity_compensation_.CoG_.force = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.selected_axes_.x = param.as_bool();
        }
        if (param.get_name() == "y") {
          admittance_.selected_axes_.y = param.as_bool();
        }
        if (param.get_name() == "z") {
          admittance_.selected_axes_.z = param.as_bool();
        }
        if (param.get_name() == "rx") {
          admittance_.selected_axes_.rx = param.as_bool();
        }
        if (param.get_name() == "ry") {
          admittance_.selected_axes_.ry = param.as_bool();
        }
        if (param.get_name() == "rz") {
          admittance_.selected_axes_.rz = param.as_bool();
        }
        if (param.get_name() == "x") {
          admittance_.mass_.x = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.mass_.y = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.mass_.z = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.mass_.rx = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.mass_.ry = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.mass_.rz = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.damping_ratio_.x = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.damping_ratio_.y = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.damping_ratio_.z = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.damping_ratio_.rx = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.damping_ratio_.ry = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.damping_ratio_.rz = param.as_double();
        }
        if (param.get_name() == "x") {
          admittance_.stiffness_.x = param.as_double();
        }
        if (param.get_name() == "y") {
          admittance_.stiffness_.y = param.as_double();
        }
        if (param.get_name() == "z") {
          admittance_.stiffness_.z = param.as_double();
        }
        if (param.get_name() == "rx") {
          admittance_.stiffness_.rx = param.as_double();
        }
        if (param.get_name() == "ry") {
          admittance_.stiffness_.ry = param.as_double();
        }
        if (param.get_name() == "rz") {
          admittance_.stiffness_.rz = param.as_double();
        }
        if (param.get_name() == "enable_parameter_update_without_reactivation") {
          enable_parameter_update_without_reactivation = param.as_bool();
        }
        if (param.get_name() == "joint_limiter_type") {
          joint_limiter_type = param.as_string();
        }
        if (param.get_name() == "state_publish_rate") {
          state_publish_rate = param.as_double();
        }

      }
      return result;
    }

    void declare_params(const std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> &parameters_interface) {
      auto p_joints = rclcpp::ParameterValue(joints);
      parameters_interface->declare_parameter("joints", p_joints);
      auto p_command_interfaces = rclcpp::ParameterValue(command_interfaces);
      parameters_interface->declare_parameter("command_interfaces", p_command_interfaces);
      auto p_state_interfaces = rclcpp::ParameterValue(state_interfaces);
      parameters_interface->declare_parameter("state_interfaces", p_state_interfaces);
      auto p_chainable_command_interfaces = rclcpp::ParameterValue(chainable_command_interfaces);
      parameters_interface->declare_parameter("chainable_command_interfaces", p_chainable_command_interfaces);
      auto p_kinematics_plugin_name = rclcpp::ParameterValue(kinematics_.plugin_name);
      parameters_interface->declare_parameter("kinematics.plugin_name", p_kinematics_plugin_name);
      auto p_kinematics_base = rclcpp::ParameterValue(kinematics_.base);
      parameters_interface->declare_parameter("kinematics.base", p_kinematics_base);
      auto p_kinematics_tip = rclcpp::ParameterValue(kinematics_.tip);
      parameters_interface->declare_parameter("kinematics.tip", p_kinematics_tip);
      auto p_kinematics_group_name = rclcpp::ParameterValue(kinematics_.group_name);
      parameters_interface->declare_parameter("kinematics.group_name", p_kinematics_group_name);
      auto p_ft_sensor_name = rclcpp::ParameterValue(ft_sensor_.name);
      parameters_interface->declare_parameter("ft_sensor.name", p_ft_sensor_name);
      auto p_ft_sensor_frame_id = rclcpp::ParameterValue(ft_sensor_.frame_id);
      parameters_interface->declare_parameter("ft_sensor.frame_id", p_ft_sensor_frame_id);
      auto p_ft_sensor_frame_external = rclcpp::ParameterValue(ft_sensor_.frame_external);
      parameters_interface->declare_parameter("ft_sensor.frame_external", p_ft_sensor_frame_external);
      auto p_control_open_loop_control = rclcpp::ParameterValue(control_.open_loop_control);
      parameters_interface->declare_parameter("control.open_loop_control", p_control_open_loop_control);
      auto p_control_frame_id = rclcpp::ParameterValue(control_.frame_id);
      parameters_interface->declare_parameter("control.frame_id", p_control_frame_id);
      auto p_control_frame_external = rclcpp::ParameterValue(control_.frame_external);
      parameters_interface->declare_parameter("control.frame_external", p_control_frame_external);
      auto p_fixed_world_frame_id = rclcpp::ParameterValue(fixed_world_frame_.id);
      parameters_interface->declare_parameter("fixed_world_frame.id", p_fixed_world_frame_id);
      auto p_fixed_world_frame_external = rclcpp::ParameterValue(fixed_world_frame_.external);
      parameters_interface->declare_parameter("fixed_world_frame.external", p_fixed_world_frame_external);
      auto p_gravity_compensation_frame_id = rclcpp::ParameterValue(gravity_compensation_.frame_id);
      parameters_interface->declare_parameter("gravity_compensation.frame_id", p_gravity_compensation_frame_id);
      auto p_gravity_compensation_external = rclcpp::ParameterValue(gravity_compensation_.external);
      parameters_interface->declare_parameter("gravity_compensation.external", p_gravity_compensation_external);
      auto p_gravity_compensation_CoG_x = rclcpp::ParameterValue(gravity_compensation_.CoG_.x);
      parameters_interface->declare_parameter("gravity_compensation.CoG.x", p_gravity_compensation_CoG_x);
      auto p_gravity_compensation_CoG_y = rclcpp::ParameterValue(gravity_compensation_.CoG_.y);
      parameters_interface->declare_parameter("gravity_compensation.CoG.y", p_gravity_compensation_CoG_y);
      auto p_gravity_compensation_CoG_z = rclcpp::ParameterValue(gravity_compensation_.CoG_.z);
      parameters_interface->declare_parameter("gravity_compensation.CoG.z", p_gravity_compensation_CoG_z);
      auto p_gravity_compensation_CoG_force = rclcpp::ParameterValue(gravity_compensation_.CoG_.force);
      parameters_interface->declare_parameter("gravity_compensation.CoG.force", p_gravity_compensation_CoG_force);
      auto p_admittance_selected_axes_x = rclcpp::ParameterValue(admittance_.selected_axes_.x);
      parameters_interface->declare_parameter("admittance.selected_axes.x", p_admittance_selected_axes_x);
      auto p_admittance_selected_axes_y = rclcpp::ParameterValue(admittance_.selected_axes_.y);
      parameters_interface->declare_parameter("admittance.selected_axes.y", p_admittance_selected_axes_y);
      auto p_admittance_selected_axes_z = rclcpp::ParameterValue(admittance_.selected_axes_.z);
      parameters_interface->declare_parameter("admittance.selected_axes.z", p_admittance_selected_axes_z);
      auto p_admittance_selected_axes_rx = rclcpp::ParameterValue(admittance_.selected_axes_.rx);
      parameters_interface->declare_parameter("admittance.selected_axes.rx", p_admittance_selected_axes_rx);
      auto p_admittance_selected_axes_ry = rclcpp::ParameterValue(admittance_.selected_axes_.ry);
      parameters_interface->declare_parameter("admittance.selected_axes.ry", p_admittance_selected_axes_ry);
      auto p_admittance_selected_axes_rz = rclcpp::ParameterValue(admittance_.selected_axes_.rz);
      parameters_interface->declare_parameter("admittance.selected_axes.rz", p_admittance_selected_axes_rz);
      auto p_admittance_mass_x = rclcpp::ParameterValue(admittance_.mass_.x);
      parameters_interface->declare_parameter("admittance.mass.x", p_admittance_mass_x);
      auto p_admittance_mass_y = rclcpp::ParameterValue(admittance_.mass_.y);
      parameters_interface->declare_parameter("admittance.mass.y", p_admittance_mass_y);
      auto p_admittance_mass_z = rclcpp::ParameterValue(admittance_.mass_.z);
      parameters_interface->declare_parameter("admittance.mass.z", p_admittance_mass_z);
      auto p_admittance_mass_rx = rclcpp::ParameterValue(admittance_.mass_.rx);
      parameters_interface->declare_parameter("admittance.mass.rx", p_admittance_mass_rx);
      auto p_admittance_mass_ry = rclcpp::ParameterValue(admittance_.mass_.ry);
      parameters_interface->declare_parameter("admittance.mass.ry", p_admittance_mass_ry);
      auto p_admittance_mass_rz = rclcpp::ParameterValue(admittance_.mass_.rz);
      parameters_interface->declare_parameter("admittance.mass.rz", p_admittance_mass_rz);
      auto p_admittance_damping_ratio_x = rclcpp::ParameterValue(admittance_.damping_ratio_.x);
      parameters_interface->declare_parameter("admittance.damping_ratio.x", p_admittance_damping_ratio_x);
      auto p_admittance_damping_ratio_y = rclcpp::ParameterValue(admittance_.damping_ratio_.y);
      parameters_interface->declare_parameter("admittance.damping_ratio.y", p_admittance_damping_ratio_y);
      auto p_admittance_damping_ratio_z = rclcpp::ParameterValue(admittance_.damping_ratio_.z);
      parameters_interface->declare_parameter("admittance.damping_ratio.z", p_admittance_damping_ratio_z);
      auto p_admittance_damping_ratio_rx = rclcpp::ParameterValue(admittance_.damping_ratio_.rx);
      parameters_interface->declare_parameter("admittance.damping_ratio.rx", p_admittance_damping_ratio_rx);
      auto p_admittance_damping_ratio_ry = rclcpp::ParameterValue(admittance_.damping_ratio_.ry);
      parameters_interface->declare_parameter("admittance.damping_ratio.ry", p_admittance_damping_ratio_ry);
      auto p_admittance_damping_ratio_rz = rclcpp::ParameterValue(admittance_.damping_ratio_.rz);
      parameters_interface->declare_parameter("admittance.damping_ratio.rz", p_admittance_damping_ratio_rz);
      auto p_admittance_stiffness_x = rclcpp::ParameterValue(admittance_.stiffness_.x);
      parameters_interface->declare_parameter("admittance.stiffness.x", p_admittance_stiffness_x);
      auto p_admittance_stiffness_y = rclcpp::ParameterValue(admittance_.stiffness_.y);
      parameters_interface->declare_parameter("admittance.stiffness.y", p_admittance_stiffness_y);
      auto p_admittance_stiffness_z = rclcpp::ParameterValue(admittance_.stiffness_.z);
      parameters_interface->declare_parameter("admittance.stiffness.z", p_admittance_stiffness_z);
      auto p_admittance_stiffness_rx = rclcpp::ParameterValue(admittance_.stiffness_.rx);
      parameters_interface->declare_parameter("admittance.stiffness.rx", p_admittance_stiffness_rx);
      auto p_admittance_stiffness_ry = rclcpp::ParameterValue(admittance_.stiffness_.ry);
      parameters_interface->declare_parameter("admittance.stiffness.ry", p_admittance_stiffness_ry);
      auto p_admittance_stiffness_rz = rclcpp::ParameterValue(admittance_.stiffness_.rz);
      parameters_interface->declare_parameter("admittance.stiffness.rz", p_admittance_stiffness_rz);
      auto p_enable_parameter_update_without_reactivation = rclcpp::ParameterValue(
          enable_parameter_update_without_reactivation);
      parameters_interface->declare_parameter("enable_parameter_update_without_reactivation",
                                              p_enable_parameter_update_without_reactivation);
      auto p_joint_limiter_type = rclcpp::ParameterValue(joint_limiter_type);
      parameters_interface->declare_parameter("joint_limiter_type", p_joint_limiter_type);
      auto p_state_publish_rate = rclcpp::ParameterValue(state_publish_rate);
      parameters_interface->declare_parameter("state_publish_rate", p_state_publish_rate);

    }
  };


} // namespace admittance_controller
