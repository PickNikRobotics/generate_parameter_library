// Copyright 2022 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "admittance_controller_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("admittance_controller") {
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
    param_listener = std::make_shared<admittance_controller::ParamListener>(
        get_node_parameters_interface());
    params_ = param_listener->get_params();
    RCLCPP_INFO(this->get_logger(), "Initial control frame parameter is: '%s'",
                params_.control.frame.id.c_str());
  }

 private:
  void timer_callback() {
    if (param_listener->is_old(params_)) {
      param_listener->refresh_dynamic_parameters();
      params_ = param_listener->get_params();
      RCLCPP_INFO(this->get_logger(), "New control frame parameter is: '%s'",
                  params_.control.frame.id.c_str());
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<admittance_controller::ParamListener> param_listener;
  admittance_controller::Params params_;
};

int main(int numArgs, const char** args) {
  rclcpp::init(numArgs, args);
  auto publisher_node = std::make_shared<MinimalPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher_node);
  executor.spin();

  return 0;
}
