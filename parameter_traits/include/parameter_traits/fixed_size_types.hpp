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

#include <string_view>

#include <tcb_span/span.hpp>

namespace parameter_traits {

template <typename T, size_t S>
class FixedSizeArray {
 public:
  FixedSizeArray() = default;
  FixedSizeArray(const std::vector<T>& values) {
    len_ = std::min(values.size(), S);
    std::copy(values.cbegin(), values.cbegin() + len_, data_.begin());
  }

  operator tcb::span<T>() { return tcb::span<T>(data_.data(), len_); }

  operator rclcpp::ParameterValue() const {
    return rclcpp::ParameterValue(
        std::vector<T>(data_.cbegin(), data_.cbegin() + len_));
  }

 private:
  std::array<T, S> data_;
  size_t len_;
};

template <size_t S>
class FixedSizeString {
 public:
  FixedSizeString() = default;
  FixedSizeString(const std::string& str) {
    len_ = std::min(str.size(), S);
    std::copy(str.cbegin(), str.cbegin() + len_, data_.begin());
  }

  operator std::string_view() const {
    return std::string_view(data_.data(), len_);
  }

  operator rclcpp::ParameterValue() const {
    return rclcpp::ParameterValue(
        std::string(data_.cbegin(), data_.cbegin() + len_));
  }

 private:
  std::array<char, S> data_;
  size_t len_;
};

}  // namespace parameter_traits
