class Result {
 public:
  template <typename... Args>
  Result(const std::string& format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) +
                 1;  // Extra space for '\0'
    if (size_s <= 0) {
      throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    std::unique_ptr<char[]> buf(new char[size]);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    msg_ = std::string(buf.get(),
                       buf.get() + size - 1);  // We don't want the '\0' inside
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

template <typename T>
Result validate_len(const rclcpp::Parameter& parameter, size_t size) {
  auto param_value = parameter.get_value<std::vector<T>>();
  if (param_value.size() != size) {
    return ERROR("Invalid length '%d' for parameter %s. Required length: %d",
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
          "Invalid value '%s' for parameter %s. Required bounds: [%s, %s]", "TODO: print value",
          parameter.get_name().c_str(), "TODO: print value", "TODO: print value");
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
    return ERROR("The parameter (%s) with the value (%s) not in the set: [%s]",
                 parameter.get_name(), "TODO: print value",
                 ss.str().c_str());
  }

  return OK;
}
