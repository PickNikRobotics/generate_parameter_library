
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "variant"
#include <optional>

namespace gen_param_struct_validators {

  class Result {
  public:
    template<typename ... Args>
    Result(const std::string &format, Args ... args) {
      int size_s = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; // Extra space for '\0'
      if (size_s <= 0) { throw std::runtime_error("Error during formatting."); }
      auto size = static_cast<size_t>( size_s );
      std::unique_ptr<char[]> buf(new char[size]);
      std::snprintf(buf.get(), size, format.c_str(), args ...);
      msg_ = std::string(buf.get(), buf.get() + size - 1); // We don't want the '\0' inside
      success_ = false;
    }

    Result() {
      success_ = true;
    }

    bool success() {
      return success_;
    }

    std::string error_msg() {
      return msg_;
    }

  private:
    std::string msg_;
    bool success_;
  };

  auto OK = Result();
  using ERROR = Result;

  Result validate_string_array_len(const rclcpp::Parameter &parameter, size_t size) {
    const auto &string_array = parameter.as_string_array();
    if (string_array.size() != size) {
      return ERROR(
          "Invalid length '%d' for parameter %s. Required length: %d", string_array.size(), parameter.get_name().c_str(),
          size);
    }
    return OK;
  }

  Result validate_double_array_len(const rclcpp::Parameter &parameter, size_t size) {
    const auto &double_array = parameter.as_double_array();
    if (double_array.size() != size) {
      return ERROR(
          "Invalid length '%d' for parameter %s. Required length: %d", double_array.size(), parameter.get_name().c_str(),
          size);
    }
    return OK;
  }

  Result validate_bool_array_len(const rclcpp::Parameter &parameter, size_t size) {
    const auto &bool_array = parameter.as_bool_array();
    if (bool_array.size() != size) {
      return ERROR(
          "Invalid length '%d' for parameter %s. Required length: %d", bool_array.size(), parameter.get_name().c_str(),
          size);
    }
    return OK;
  }

  Result validate_double_array_bounds(const rclcpp::Parameter &parameter, double lower_bound, double upper_bound) {
    const auto &double_array = parameter.as_double_array();
    for (auto val: double_array) {
      if (val < lower_bound || val > upper_bound) {
        return ERROR(
            "Invalid value '%f' for parameter %s. Required bounds: [%f, %f]",
            val, parameter.get_name().c_str(), lower_bound, upper_bound);
      }
    }
    return OK;
  }

  Result validate_int_array_bounds(const rclcpp::Parameter &parameter, int lower_bound, int upper_bound) {
    const auto &integer_array = parameter.as_integer_array();
    for (auto val: integer_array) {
      if (val < lower_bound || val > upper_bound) {
        return ERROR(
            "Invalid value '%d' for parameter %s. Required bounds: [%d, %d]",
            val, parameter.get_name().c_str(), lower_bound, upper_bound);
      }
    }
    return OK;
  }

} // namespace gen_param_struct

