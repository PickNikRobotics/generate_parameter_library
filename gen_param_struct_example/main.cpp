#include "config/admittance_controller.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("admittance_controller") {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  std::shared_ptr<admittance_controller_parameters::admittance_controller> gen_struct;

private:
  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "Control frame is: '%s'", gen_struct->params_.control_.frame_.id_.c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int numArgs, const char **args) {

  rclcpp::init(numArgs, args);

  auto node = std::make_shared<MinimalPublisher>();
  auto gen_struct = std::make_shared<admittance_controller_parameters::admittance_controller>(
      node->get_node_parameters_interface());
  node->gen_struct = gen_struct;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}


