# Migration notes for generate_parameter_library

0.2.6 to main
-------------
* Header `parameter_traits/result.hpp` removed, code moved into `parameter_traits/validators.hpp`
* `parameter_traits::Result` renamed to `parameter_traits::ValidateResult`
* `parameter_traits::ERROR` renamed to `parameter_traits::make_error`
* `parameter_traits::OK` renamed to `parameter_traits::ok`
