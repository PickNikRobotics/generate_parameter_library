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

#include <functional>
#include <string>
#include <type_traits>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <parameter_traits/comparison.hpp>
#include <parameter_traits/result.hpp>

namespace parameter_traits {

template <typename T>
Result unique(rclcpp::Parameter const& parameter) {
  if (!is_unique<T>(parameter.get_value<std::vector<T>>())) {
    return ERROR("Parameter '{}' must only contain unique values",
                 parameter.get_name());
  }
  return OK;
}

template <typename T>
Result subset_of(rclcpp::Parameter const& parameter,
                 std::vector<T> valid_values) {
  auto const& input_values = parameter.get_value<std::vector<T>>();

  for (auto const& value : input_values) {
    if (!contains(valid_values, value)) {
      return ERROR("Invalid entry '{}' for parameter '{}'. Not in set: {}",
                   value, parameter.get_name(), valid_values);
    }
  }

  return OK;
}

template <typename T, typename F>
Result size_cmp(rclcpp::Parameter const& parameter, size_t size,
                std::string const& cmp_str, F cmp) {
  if (std::is_same<T, std::string>::value) {
    if (auto value = parameter.get_value<std::string>();
        !cmp(value.size(), size)) {
      return ERROR("Invalid length '{}' for parameter '{}'. Required {}: {}",
                   value.size(), parameter.get_name(), cmp_str, size);
    }
  } else {
    if (auto value = parameter.get_value<std::vector<T>>();
        !cmp(value.size(), size)) {
      return ERROR("Invalid length '{}' for parameter '{}'. Required {}: {}",
                   value.size(), parameter.get_name(), cmp_str, size);
    }
  }

  return OK;
}

template <typename T>
Result fixed_size(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "equal to", std::equal_to<size_t>{});
}

template <typename T>
Result size_gt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "greater than", std::greater<size_t>{});
}

template <typename T>
Result size_lt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "less than", std::less<size_t>{});
}

template <typename T>
Result not_empty(rclcpp::Parameter const& parameter) {
  if (std::is_same<T, std::string>::value) {
    if (auto param_value = parameter.get_value<std::string>();
        param_value.empty()) {
      return ERROR("The parameter '{}' cannot be empty.", parameter.get_name());
    }
  } else {
    if (auto param_value = parameter.get_value<std::vector<T>>();
        param_value.empty()) {
      return ERROR("The parameter '{}' cannot be empty.", parameter.get_name());
    }
  }
  return OK;
}

template <typename T>
Result element_bounds(const rclcpp::Parameter& parameter, T lower, T upper) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower || val > upper) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
          val, parameter.get_name(), lower, upper);
    }
  }
  return OK;
}

template <typename T>
Result lower_element_bounds(const rclcpp::Parameter& parameter, T lower) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
          val, parameter.get_name(), lower);
    }
  }
  return OK;
}

template <typename T>
Result upper_element_bounds(const rclcpp::Parameter& parameter, T upper) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val > upper) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
          val, parameter.get_name(), upper);
    }
  }
  return OK;
}

template <typename T>
Result bounds(const rclcpp::Parameter& parameter, T lower, T upper) {
  auto param_value = parameter.get_value<T>();
  if (param_value < lower || param_value > upper) {
    return ERROR(
        "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
        param_value, parameter.get_name(), lower, upper);
  }
  return OK;
}

template <typename T>
Result lower_bounds(const rclcpp::Parameter& parameter, T lower) {
  auto param_value = parameter.get_value<T>();
  if (param_value < lower) {
    return ERROR(
        "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
        param_value, parameter.get_name(), lower);
  }
  return OK;
}

template <typename T>
Result upper_bounds(const rclcpp::Parameter& parameter, T upper) {
  auto param_value = parameter.get_value<T>();
  if (param_value > upper) {
    return ERROR(
        "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
        param_value, parameter.get_name(), upper);
  }
  return OK;
}

template <typename T>
Result one_of(rclcpp::Parameter const& parameter, std::vector<T> collection) {
  auto param_value = parameter.get_value<T>();

  if (std::find(collection.cbegin(), collection.cend(), param_value) ==
      collection.end()) {
    return ERROR("The parameter '{}' with the value '{}' not in the set: {}",
                 parameter.get_name(), param_value,
                 fmt::format("{}", fmt::join(collection, ", ")));
  }
  return OK;
}

}  // namespace parameter_traits
