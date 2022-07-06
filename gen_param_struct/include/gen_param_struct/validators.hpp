//
// Created by paul on 7/5/22.
//

#ifndef COLCON_WS_VALIDATORS_H
#define COLCON_WS_VALIDATORS_H

#include "rclcpp/rclcpp.hpp"
#include "variant"
#include <optional>

class Result {
public:
  Result(const char* fmt, ...) {
    const char* tmp = fmt;
    int counter = 0;
    while (tmp){
      counter+= ('%' == *tmp++);
    }
    va_list args;
    va_start(args, fmt);
    int buffer_size = 1;
    for (int i=0; i < counter; i++){
      buffer_size += strlen(va_arg(args, char*));
    }
    va_end(args);
    va_start(args, fmt);
    std::vector<char> buffer(strlen(fmt) + buffer_size);
    sprintf(buffer.data(), fmt, args);
    msg_ = std::string(buffer.data());
  }

  Result() {
    success_ = true;
  }

  bool success(){
    return success_;
  }

  std::string error_msg(){
    return msg_;
  }

private:
  std::string msg_;
  bool success_;
};

auto OK = Result();
using ERROR = Result;

Result validate_string_array_len(const rclcpp::Parameter& parameter, size_t size) {
  const auto& string_array = parameter.as_string_array();
  if (string_array.size() != size) {
    return ERROR(
        "The length of the parameter was incorrect. Expected size is %s, actual size is %s",
        size, string_array.size());
  }
  return OK;
}

Result validate_double_array_len(const rclcpp::Parameter& parameter, size_t size) {
  const auto& double_array = parameter.as_double_array();
  if (double_array.size() != size) {
    return ERROR(
        "The length of the parameter was incorrect. Expected size is %s, actual size is %s",
        size, double_array.size());
  }
  return OK;
}

Result validate_bool_array_len(const rclcpp::Parameter& parameter, size_t size) {
  const auto& bool_array = parameter.as_bool_array();
  if (bool_array.size() != size) {
    return ERROR(
        "The length of the parameter was incorrect. Expected size is %s, actual size is %s",
        size, bool_array.size());
  }
  return OK;
}

Result validate_double_array_bounds(const rclcpp::Parameter& parameter, double lower_bound, double upper_bound) {
  const auto &double_array = parameter.as_double_array();
  for (auto val: double_array) {
    if (val < lower_bound || val > upper_bound) {
      return ERROR(
          "The parameter value (%f) was outside the allowed bounds [(%f), (%f)]",
          val, lower_bound, upper_bound);
    }

    return OK;
  }
}


//template<typename T>
//bool validate_length(const std::string& name, const std::vector<T>& values, size_t len,  rcl_interfaces::msg::SetParametersResult& result){
//  if (!validate_length(values, len)){
//    result.reason = std::string("Invalid size for vector parameter ") + name +
//                    ". Expected " + std::to_string(len) + " got " + std::to_string(values.size());
//    return false;
//  }
//  return true;
//}


#endif //COLCON_WS_VALIDATORS_H
