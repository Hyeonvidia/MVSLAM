#pragma once
// Analytic test trajectory with exact IMU measurements (body rates + specific force) for the
// preintegration / inertial factor / initialization tests.
#include <cmath>
#include <vector>

#include "core/math/se3.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam::testing {

struct ImuTrajectory {
  core::Vec3 omega{0.1, 0.2, 0.3};        // constant body angular velocity -> R(t) = Exp(omega t)
  core::Vec3 g_w{0, 0, -9.81};
  core::Vec6 bias = core::Vec6::Zero();    // [bg, ba] added to the measurements
  core::SO3 R(double t) const { return core::SO3::exp(omega * t); }
  core::Vec3 p(double t) const { return {std::sin(t), 0.5 * std::cos(2 * t), 0.3 * t + 0.2 * std::sin(3 * t)}; }
  core::Vec3 v(double t) const { return {std::cos(t), -std::sin(2 * t), 0.3 + 0.6 * std::cos(3 * t)}; }
  core::Vec3 a(double t) const { return {-std::sin(t), -2 * std::cos(2 * t), -1.8 * std::sin(3 * t)}; }
  core::SE3 T(double t) const { return core::SE3(R(t), p(t)); }
  // Measurements at time t: gyro = omega + bg, acc = R^T (a - g) + ba.
  core::Vec3 gyro(double t) const { (void)t; return omega + bias.head<3>(); }
  core::Vec3 acc(double t) const { return R(t).inverse() * (a(t) - g_w) + bias.tail<3>(); }
  // Samples on [t0, t1] at `rate` Hz (inclusive of both ends).
  std::vector<ImuSample> samples(double t0, double t1, double rate) const {
    std::vector<ImuSample> out;
    const int n = static_cast<int>(std::round((t1 - t0) * rate));
    for (int i = 0; i <= n; ++i) {
      const double t = t0 + i / rate;
      out.push_back({static_cast<Timestamp>(std::llround(t * 1e9)), core::fromVec3(gyro(t)), core::fromVec3(acc(t))});
    }
    return out;
  }
};

}  // namespace mvslam::testing
