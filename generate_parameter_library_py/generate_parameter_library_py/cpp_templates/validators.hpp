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
Result validate_len(const rclcpp::Parameter& parameter, size_t size) {
  auto param_value = parameter.get_value<std::vector<T>>();
  if (param_value.size() != size) {
    return ERROR("Invalid length '{}' for parameter {}. Required length: {}",
                 param_value.size(), parameter.get_name().c_str(), size);
  }
  return OK;
}

template <typename T>
Result validate_bounds(const rclcpp::Parameter& parameter,
                                 T lower_bound, T upper_bound) {
  auto param_value = parameter.get_value<std::vector<T>>();
  for (auto val : param_value) {
    if (val < lower_bound || val > upper_bound) {
      return ERROR(
          "Invalid value '{}' for parameter '{}'. Required bounds: [{}, {}]",
          val, parameter.get_name(), lower_bound, upper_bound);
    }
  }
  return OK;
}

template <typename T>
Result validate_one_of(rclcpp::Parameter const& parameter,
                       std::vector<T> collection) {
  auto param_value = parameter.get_value<T>();

  if (std::find(collection.cbegin(), collection.cend(), param_value) ==
      collection.end()) {
    return ERROR("The parameter '{}' with the value '{}' not in the set: {}",
                 parameter.get_name(), param_value,
                 fmt::format("{}", fmt::join(collection, ", ")));
  }

  return OK;
}
