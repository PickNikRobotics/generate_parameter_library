// Copyright 2025 Forssea Robotics
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
//    * Neither the name of Forssea Robotics nor the names of its
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

#include "generate_parameter_library_example_external/minimal_publisher_external.hpp"

#include <rclcpp/rclcpp.hpp>

#include <generate_parameter_library_example/admittance_controller_parameters.hpp>

using namespace std::chrono_literals;

namespace admittance_controller {

MinimalPublisher::MinimalPublisher(const rclcpp::NodeOptions& options)
    : Node("admittance_controller", options) {
  timer_ = create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  param_listener_ =
      std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  [[maybe_unused]] StackParams s_params = param_listener_->get_stack_params();

  RCLCPP_INFO(get_logger(), "Initial control frame parameter is: '%s'",
              params_.control.frame.id.c_str());
  RCLCPP_INFO(get_logger(), "fixed string is: '%s'",
              std::string{params_.fixed_string}.c_str());
  const tcb::span<double> fixed_array = params_.fixed_array;
  for (auto d : fixed_array) {
    RCLCPP_INFO(get_logger(), "value: '%s'", std::to_string(d).c_str());
  }
}

void MinimalPublisher::timer_callback() {
  if (param_listener_->is_old(params_)) {
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_logger(), "New control frame parameter is: '%s'",
                params_.control.frame.id.c_str());
    RCLCPP_INFO(get_logger(), "fixed string is: '%s'",
                std::string{params_.fixed_string}.c_str());
    const tcb::span<double> fixed_array = params_.fixed_array;
    for (auto d : fixed_array) {
      RCLCPP_INFO(get_logger(), "value: '%s'", std::to_string(d).c_str());
    }
  }
}

}  // namespace admittance_controller
