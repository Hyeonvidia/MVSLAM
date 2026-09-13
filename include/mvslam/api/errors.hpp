#pragma once
// Error policy class 1 (API misuse): thrown from constructors / validate(). Never thrown by track().
#include <stdexcept>
#include <string>

namespace mvslam {
struct ConfigError : std::invalid_argument { using std::invalid_argument::invalid_argument; };
struct InitError : std::runtime_error { using std::runtime_error::runtime_error; };
}  // namespace mvslam
