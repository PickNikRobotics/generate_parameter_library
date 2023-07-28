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

#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <tl_expected/expected.hpp>

namespace custom_validators {

// User defined parameter validation
tl::expected<void, std::string> validate_double_array_custom_func(
    const rclcpp::Parameter& parameter, double max_sum, double max_element) {
  const auto& double_array = parameter.as_double_array();
  double sum = 0.0;
  for (auto val : double_array) {
    sum += val;
    if (val > max_element) {
      return tl::make_unexpected(fmt::format(
          "The parameter contained an element greater than the max allowed "
          "value.  (%f) was greater than (%f)",
          val, max_element));
    }
  }
  if (sum > max_sum) {
    return tl::make_unexpected(fmt::format(
        "The sum of the parameter vector was greater than the max allowed "
        "value.  (%f) was greater than (%f)",
        sum, max_sum));
  }

  return {};
}

tl::expected<void, std::string> no_args_validator(const rclcpp::Parameter&) {
  return {};
}

}  // namespace custom_validators
