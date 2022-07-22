class Result {
 public:
  template <typename... Args>
  Result(const std::string& format, Args... args) {
    msg_ = fmt::format(format, args...);
    success_ = false;
  }

  Result() = default;

  operator rcl_interfaces::msg::SetParametersResult() const {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = success_;
    result.reason = msg_;
    return result;
  }

  bool success() { return success_; }

  std::string error_msg() { return msg_; }

 private:
  std::string msg_;
  bool success_ = true;
};

auto OK = Result();
using ERROR = Result;

template <typename T>
bool contains(std::vector<T> const& vec, T const& val) {
  return std::find(vec.cbegin(), vec.cend(), val) != vec.cend();
}

template <class T>
bool is_unique(std::vector<T> const& x) {
  auto vec = x;
  std::sort(vec.begin(), vec.end());
  return std::adjacent_find(vec.cbegin(), vec.cend()) == vec.cend();
}

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

template <typename T>
Result fixed_size(const rclcpp::Parameter& parameter, size_t size) {
  auto param_value = parameter.get_value<std::vector<T>>();
  if (param_value.size() != size) {
    return ERROR("Invalid length '{}' for parameter '{}'. Required length: {}",
                 param_value.size(), parameter.get_name().c_str(), size);
  }
  return OK;
}

template <typename T>
Result size_gt(rclcpp::Parameter const& parameter, size_t size) {
  auto const& values = parameter.get_value<std::vector<T>>();
  if (values.size() > size) {
    return OK;
  }

  return ERROR(
      "Invalid length '{}' for parameter '{}'. Required greater than: {}",
      values.size(), parameter.get_name(), size);
}

template <typename T>
Result size_lt(rclcpp::Parameter const& parameter, size_t size) {
  auto const& values = parameter.get_value<std::vector<T>>();
  if (values.size() < size) {
    return OK;
  }

  return ERROR("Invalid length '{}' for parameter '{}'. Required less than: {}",
               values.size(), parameter.get_name(), size);
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
