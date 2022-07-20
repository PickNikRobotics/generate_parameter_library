
// User defined parameter validation
Result validate_double_array_custom_func(const rclcpp::Parameter& parameter,
                                         double max_sum, double max_element) {
  const auto& double_array = parameter.as_double_array();
  double sum = 0.0;
  for (auto val : double_array) {
    sum += val;
    if (val > max_element) {
      return ERROR(
          "The parameter contained an element greater than the max allowed "
          "value.  (%f) was greater than (%f)",
          val, max_element);
    }
  }
  if (sum > max_sum) {
    return ERROR(
        "The sum of the parameter vector was greater than the max allowed "
        "value.  (%f) was greater than (%f)",
        sum, max_sum);
  }

  return OK;
}
