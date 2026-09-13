#pragma once
// Backend-side name for the IMU preintegration types (implemented in sensors/imu so the map can hold
// them). docs/design/03-backend.md §4.
#include "sensors/imu/preintegration.hpp"

namespace mvslam::backend::imu {
using Preintegration = sensors::imu::Preintegration;
using Preintegrator = sensors::imu::Preintegrator;
using Measurement = sensors::imu::Measurement;
}  // namespace mvslam::backend::imu
