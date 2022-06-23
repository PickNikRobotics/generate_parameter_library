#include <iostream>
#include "parameters/admittance_controller.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std::chrono_literals;


class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher() : Node("minimal_publisher")
  {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  std::shared_ptr<admittance_controller_parameters::admittance_controller> gen_struct;

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Joint 0 is: '%s'", gen_struct->joints[0].c_str());
  }

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int numArgs, const char** args) {

  rclcpp::init(numArgs, args);

  auto node = std::make_shared<MinimalPublisher>();
  auto gen_struct = std::make_shared<admittance_controller_parameters::admittance_controller>(node->get_node_parameters_interface());
  node->gen_struct = gen_struct;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}


