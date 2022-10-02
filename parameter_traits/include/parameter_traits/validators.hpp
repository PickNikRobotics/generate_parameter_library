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

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <parameter_traits/comparison.hpp>
#include <tl_expected/expected.hpp>

namespace parameter_traits {

using ValidateResult = tl::expected<void, std::string>;
using ok = ValidateResult;

template <typename... Args>
ValidateResult make_error(std::string const& format, Args... args) {
  return tl::make_unexpected(fmt::format(format, args...));
}

rcl_interfaces::msg::SetParametersResult to_set_parameters_result(
    ValidateResult const& res);

template <typename T>
ValidateResult unique(rclcpp::Parameter const& parameter) {
  if (!is_unique<T>(parameter.get_value<std::vector<T>>())) {
    return make_error("Parameter '{}' must only contain unique values",
                      parameter.get_name());
  }
  return ok();
}

template <typename T>
ValidateResult subset_of(rclcpp::Parameter const& parameter,
                         std::vector<T> valid_values) {
  auto const& input_values = parameter.get_value<std::vector<T>>();

  for (auto const& value : input_values) {
    if (!contains(valid_values, value)) {
      return make_error(fmt::format(
          "Invalid entry '{}' for parameter '{}'. Not in set: {}", value,
          parameter.get_name(), fmt::join(valid_values, ", ")));
    }
  }

  return ok();
}

template <typename T, typename F>
ValidateResult size_cmp(rclcpp::Parameter const& parameter, size_t size,
                        std::string const& cmp_str, F cmp) {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    if (auto value = parameter.get_value<std::string>();
        !cmp(value.size(), size)) {
      return make_error(
          "Invalid length '{}' for parameter '{}'. Required {}: {}",
          value.size(), parameter.get_name(), cmp_str, size);
    }
  } else {
    if (auto value = parameter.get_value<std::vector<T>>();
        !cmp(value.size(), size)) {
      return make_error(
          "Invalid length '{}' for parameter '{}'. Required {}: {}",
          value.size(), parameter.get_name(), cmp_str, size);
    }
  }

  return ok();
}

template <typename T>
ValidateResult fixed_size(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "equal to", std::equal_to<size_t>{});
}

template <typename T>
ValidateResult size_gt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "greater than", std::greater<size_t>{});
}

template <typename T>
ValidateResult size_lt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "less than", std::less<size_t>{});
}

template <typename T>
ValidateResult not_empty(rclcpp::Parameter const& parameter) {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    if (auto param_value = parameter.get_value<std::string>();
        param_value.empty()) {
      return make_error("The parameter '{}' cannot be empty.",
                        parameter.get_name());
    }
  } else {
    if (auto param_value = parameter.get_value<std::vector<T>>();
        param_value.empty()) {
      return make_error("The parameter '{}' cannot be empty.",
                        parameter.get_name());
    }
  }
  return ok();
}

template <typename T>
ValidateResult element_bounds(const rclcpp::Parameter& parameter, T lower,
                              T upper) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower || val > upper) {
      return make_error(
          "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
          val, parameter.get_name(), lower, upper);
    }
  }
  return ok();
}

template <typename T>
ValidateResult lower_element_bounds(const rclcpp::Parameter& parameter,
                                    T lower) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower) {
      return make_error(
          "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
          val, parameter.get_name(), lower);
    }
  }
  return ok();
}

template <typename T>
ValidateResult upper_element_bounds(const rclcpp::Parameter& parameter,
                                    T upper) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val > upper) {
      return make_error(
          "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
          val, parameter.get_name(), upper);
    }
  }
  return ok();
}

template <typename T>
ValidateResult bounds(const rclcpp::Parameter& parameter, T lower, T upper) {
  auto param_value = parameter.get_value<T>();
  if (param_value < lower || param_value > upper) {
    return make_error(
        "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
        param_value, parameter.get_name(), lower, upper);
  }
  return ok();
}

template <typename T, typename Fn>
ValidateResult cmp(rclcpp::Parameter const& parameter, T value,
                   std::string const& cmp_str, Fn predicate) {
  if (auto const param_value = parameter.get_value<T>();
      !predicate(param_value, value)) {
    return make_error("Invalid value '{}' for parameter '{}'. Required {}: {}",
                      param_value, parameter.get_name(), cmp_str, value);
  }

  return ok();
}

template <typename T>
ValidateResult lower_bounds(rclcpp::Parameter const& parameter, T value) {
  return cmp(parameter, value, "lower bounds", std::greater_equal<T>{});
}

template <typename T>
ValidateResult upper_bounds(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "upper bounds", std::less_equal<T>{});
}

template <typename T>
ValidateResult lt(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "less than", std::less<T>{});
}

template <typename T>
ValidateResult gt(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "greater than", std::greater<T>{});
}

template <typename T>
ValidateResult lt_eq(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "less than or equal", std::less_equal<T>{});
}

template <typename T>
ValidateResult gt_eq(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "greater than or equal",
             std::greater_equal<T>{});
}

template <typename T>
ValidateResult one_of(rclcpp::Parameter const& parameter,
                      std::vector<T> collection) {
  auto param_value = parameter.get_value<T>();

  if (std::find(collection.cbegin(), collection.cend(), param_value) ==
      collection.end()) {
    return make_error(
        "The parameter '{}' with the value '{}' not in the set: {}",
        parameter.get_name(), param_value,
        fmt::format("{}", fmt::join(collection, ", ")));
  }
  return ok();
}

}  // namespace parameter_traits
