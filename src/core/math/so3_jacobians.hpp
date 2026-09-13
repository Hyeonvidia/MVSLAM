#pragma once
// SO(3) right Jacobian and its inverse (Forster et al. 2017 / ORB-SLAM3 RightJacobianSO3):
// Exp(phi + delta) ~= Exp(phi) * Exp(Jr(phi) * delta),  Log(Exp(phi) * Exp(delta)) ~= phi + Jr^{-1}(phi) * delta.
#include <cmath>

#include "core/math/se3.hpp"

namespace mvslam::core {

inline Mat3 rightJacobianSO3(const Vec3& phi) {
  const double theta2 = phi.squaredNorm();
  const Mat3 W = SO3::hat(phi);
  if (theta2 < 1e-10) return Mat3::Identity() - 0.5 * W;
  const double theta = std::sqrt(theta2);
  return Mat3::Identity() - W * (1.0 - std::cos(theta)) / theta2 + W * W * (theta - std::sin(theta)) / (theta2 * theta);
}

inline Mat3 inverseRightJacobianSO3(const Vec3& phi) {
  const double theta2 = phi.squaredNorm();
  const Mat3 W = SO3::hat(phi);
  if (theta2 < 1e-10) return Mat3::Identity() + 0.5 * W;
  const double theta = std::sqrt(theta2);
  return Mat3::Identity() + 0.5 * W + W * W * (1.0 / theta2 - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta)));
}

inline Mat3 normalizedRotation(const Mat3& R) { return Eigen::Quaterniond(R).normalized().toRotationMatrix(); }

}  // namespace mvslam::core
