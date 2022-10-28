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
#include <variant>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>
#include <rsl/algorithm.hpp>
#include <tl_expected/expected.hpp>

namespace parameter_traits {

auto to_parameter_result_msg(
    tl::expected<std::monostate, std::string> const& result) {
  rcl_interfaces::msg::SetParametersResult msg;
  msg.successful = result.has_value();
  msg.reason = result.has_value() ? "" : result.error();
  return msg;
}

auto ok() { return std::monostate{}; }
using ValidateResult = tl::expected<std::monostate, std::string>;

template <typename... Args>
auto make_validate_help(const std::string& format, Args... args)
    -> ValidateResult {
  return tl::make_unexpected(fmt::format(format, args...));
}

template <typename T>
auto unique(rclcpp::Parameter const& parameter)
    -> tl::expected<std::monostate, std::string> {
  if (!rsl::is_unique<std::vector<T>>(parameter.get_value<std::vector<T>>())) {
    return make_validate_help("Parameter '{}' must only contain unique values",
                              parameter.get_name());
  }
  return ok();
}

template <typename T>
auto subset_of(rclcpp::Parameter const& parameter, std::vector<T> valid_values)
    -> tl::expected<std::monostate, std::string> {
  auto const& input_values = parameter.get_value<std::vector<T>>();

  for (auto const& value : input_values) {
    if (!rsl::contains(valid_values, value)) {
      return make_validate_help(
          "Invalid entry '{}' for parameter '{}'. Not in set: {}", value,
          parameter.get_name(), valid_values);
    }
  }

  return ok();
}

template <typename T, typename F>
auto size_cmp(rclcpp::Parameter const& parameter, size_t size,
              std::string const& cmp_str, F cmp)
    -> tl::expected<std::monostate, std::string> {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    if (auto value = parameter.get_value<std::string>();
        !cmp(value.size(), size)) {
      return make_validate_help(
          "Invalid length '{}' for parameter '{}'. Required {}: {}",
          value.size(), parameter.get_name(), cmp_str, size);
    }
  } else {
    if (auto value = parameter.get_value<std::vector<T>>();
        !cmp(value.size(), size)) {
      return make_validate_help(
          "Invalid length '{}' for parameter '{}'. Required {}: {}",
          value.size(), parameter.get_name(), cmp_str, size);
    }
  }

  return ok();
}

template <typename T>
auto fixed_size(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "equal to", std::equal_to<size_t>{});
}

template <typename T>
auto size_gt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "greater than", std::greater<size_t>{});
}

template <typename T>
auto size_lt(rclcpp::Parameter const& parameter, size_t size) {
  return size_cmp<T>(parameter, size, "less than", std::less<size_t>{});
}

template <typename T>
auto not_empty(rclcpp::Parameter const& parameter)
    -> tl::expected<std::monostate, std::string> {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    if (auto param_value = parameter.get_value<std::string>();
        param_value.empty()) {
      return make_validate_help("The parameter '{}' cannot be empty.",
                                parameter.get_name());
    }
  } else {
    if (auto param_value = parameter.get_value<std::vector<T>>();
        param_value.empty()) {
      return make_validate_help("The parameter '{}' cannot be empty.",
                                parameter.get_name());
    }
  }
  return ok();
}

template <typename T>
auto element_bounds(const rclcpp::Parameter& parameter, T lower, T upper)
    -> tl::expected<std::monostate, std::string> {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower || val > upper) {
      return make_validate_help(
          "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
          val, parameter.get_name(), lower, upper);
    }
  }
  return ok();
}

template <typename T>
auto lower_element_bounds(const rclcpp::Parameter& parameter, T lower)
    -> tl::expected<std::monostate, std::string> {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower) {
      return make_validate_help(
          "Invalid value '{}' for parameter '{}'. Required lower bounds: {}",
          val, parameter.get_name(), lower);
    }
  }
  return ok();
}

template <typename T>
auto upper_element_bounds(const rclcpp::Parameter& parameter, T upper)
    -> tl::expected<std::monostate, std::string> {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val > upper) {
      return make_validate_help(
          "Invalid value '{}' for parameter '{}'. Required upper bounds: {}",
          val, parameter.get_name(), upper);
    }
  }
  return ok();
}

template <typename T>
auto bounds(const rclcpp::Parameter& parameter, T lower, T upper)
    -> tl::expected<std::monostate, std::string> {
  auto param_value = parameter.get_value<T>();
  if (param_value < lower || param_value > upper) {
    return make_validate_help(
        "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
        param_value, parameter.get_name(), lower, upper);
  }
  return ok();
}

template <typename T, typename Fn>
auto cmp(rclcpp::Parameter const& parameter, T value,
         std::string const& cmp_str, Fn predicate)
    -> tl::expected<std::monostate, std::string> {
  if (auto const param_value = parameter.get_value<T>();
      !predicate(param_value, value)) {
    return make_validate_help(
        "Invalid value '{}' for parameter '{}'. Required {}: {}", param_value,
        parameter.get_name(), cmp_str, value);
  }

  return ok();
}

template <typename T>
auto lower_bounds(rclcpp::Parameter const& parameter, T value) {
  return cmp(parameter, value, "lower bounds", std::greater_equal<T>{});
}

template <typename T>
auto upper_bounds(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "upper bounds", std::less_equal<T>{});
}

template <typename T>
auto lt(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "less than", std::less<T>{});
}

template <typename T>
auto gt(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "greater than", std::greater<T>{});
}

template <typename T>
auto lt_eq(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "less than or equal", std::less_equal<T>{});
}

template <typename T>
auto gt_eq(const rclcpp::Parameter& parameter, T value) {
  return cmp(parameter, value, "greater than or equal",
             std::greater_equal<T>{});
}

template <typename T>
auto one_of(rclcpp::Parameter const& parameter, std::vector<T> collection)
    -> tl::expected<std::monostate, std::string> {
  auto param_value = parameter.get_value<T>();

  if (std::find(collection.cbegin(), collection.cend(), param_value) ==
      collection.end()) {
    return make_validate_help(
        "The parameter '{}' with the value '{}' not in the set: {}",
        parameter.get_name(), param_value,
        fmt::format("{}", fmt::join(collection, ", ")));
  }
  return ok();
}

}  // namespace parameter_traits
