
#include "validators.hpp"
#include "config/admittance_controller.h"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("admittance_controller") {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    param_listener = std::make_shared<admittance_controller_parameters::ParamListener>(get_node_parameters_interface());
    params_ = param_listener->get_params();
    RCLCPP_INFO(this->get_logger(), "Initial control frame parameter is: '%s'", params_.control_.frame_.id_.c_str());
  }

private:
  void timer_callback() {
    if (param_listener->is_invalid(params_)){
      params_ = param_listener->get_params();
      RCLCPP_INFO(this->get_logger(), "New control frame parameter is: '%s'", params_.control_.frame_.id_.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<admittance_controller_parameters::ParamListener> param_listener;
  admittance_controller_parameters::Params params_;
};

int main(int numArgs, const char **args) {
  rclcpp::init(numArgs, args);
  auto publisher_node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.spin();

  return 0;
}


