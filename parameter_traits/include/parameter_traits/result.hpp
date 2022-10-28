// Copyright (c) 2022, PickNik Inc.
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
//    * Neither the name of the copyright holder nor the names of its
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

#include <fmt/format.h>

namespace parameter_traits {

class ParameterResult {
 public:
  template <typename... Args>
  ParameterResult(const std::string& format, Args... args) {
    msg_ = fmt::format(format, args...);
    success_ = false;
  }

  ParameterResult() = default;

  bool success() const { return success_; }

  std::string error_msg() const { return msg_; }
  std::string error() const { return msg_; }

  operator bool() const { return success(); }

 private:
  std::string msg_;
  bool success_ = true;
};

using Result
    [[deprecated("Use tl::expected<T, std::string> instead. #include "
                 "<tl_expected/expected.hpp>")]] = ParameterResult;

auto static OK = Result();
using ERROR
    [[deprecated("Use tl::expected<T, std::string> instead. #include "
                 "<tl_expected/expected.hpp>")]] = Result;

auto to_parameter_result_msg(Result const& result) {
  rcl_interfaces::msg::SetParametersResult msg;
  msg.successful = bool{result};
  msg.reason = result.error_msg();
  return msg;
}

}  // namespace parameter_traits
