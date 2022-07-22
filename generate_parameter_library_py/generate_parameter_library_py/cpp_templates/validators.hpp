class Result {
 public:
  template <typename... Args>
  Result(const std::string& format, Args... args) {
    msg_ = fmt::format(format, args...);
    success_ = false;
  }

  Result() { success_ = true; }

  bool success() { return success_; }

  std::string error_msg() { return msg_; }

 private:
  std::string msg_;
  bool success_;
};

auto OK = Result();
using ERROR = Result;

Result validate_string_array_len(const rclcpp::Parameter& parameter,
                                 size_t size) {
  const auto& string_array = parameter.as_string_array();
  if (string_array.size() != size) {
    return ERROR("Invalid length '{}' for parameter {}. Required length: {}",
                 string_array.size(), parameter.get_name().c_str(), size);
  }
  return OK;
}

Result validate_double_array_len(const rclcpp::Parameter& parameter,
                                 size_t size) {
  const auto& double_array = parameter.as_double_array();
  if (double_array.size() != size) {
    return ERROR("Invalid length '{}' for parameter {}. Required length: {}",
                 double_array.size(), parameter.get_name().c_str(), size);
  }
  return OK;
}

Result validate_bool_array_len(const rclcpp::Parameter& parameter,
                               size_t size) {
  const auto& bool_array = parameter.as_bool_array();
  if (bool_array.size() != size) {
    return ERROR("Invalid length '{}' for parameter {}. Required length: {}",
                 bool_array.size(), parameter.get_name().c_str(), size);
  }
  return OK;
}

Result validate_double_array_bounds(const rclcpp::Parameter& parameter,
                                    double lower_bound, double upper_bound) {
  const auto& double_array = parameter.as_double_array();
  for (auto val : double_array) {
    if (val < lower_bound || val > upper_bound) {
      return ERROR(
          "Invalid value '{}' for parameter {}. Required bounds: [{}, {}]", val,
          parameter.get_name().c_str(), lower_bound, upper_bound);
    }
  }
  return OK;
}

Result validate_int_array_bounds(const rclcpp::Parameter& parameter,
                                 int lower_bound, int upper_bound) {
  const auto& integer_array = parameter.as_integer_array();
  for (auto val : integer_array) {
    if (val < lower_bound || val > upper_bound) {
      return ERROR(
          "Invalid value '%d' for parameter {}. Required bounds: [%d, %d]", val,
          parameter.get_name().c_str(), lower_bound, upper_bound);
    }
  }
  return OK;
}

template <typename T>
Result validate_one_of(rclcpp::Parameter const& parameter,
                       std::set<T> collection) {
  auto param_value = parameter.get_value<T>();

  if (collection.find(param_value) == collection.end()) {
    std::stringstream ss;
    for (auto const& c : collection) ss << c << ", ";
    return ERROR("The parameter ({}) with the value ({}) not in the set: [{}]",
                 parameter.get_name(), param_value, ss.str().c_str());
  }

  return OK;
}

// using validate_string_one_of = validate_one_of<std::string>;
// using validate_int_one_of = validate_one_of<int>;
