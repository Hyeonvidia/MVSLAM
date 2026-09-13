#pragma once
// Internal geometry aliases (Sophus/Eigen). Never included from public headers.
// Conventions: T_wb body->world; tangent vectors are [translation(3), rotation(3)] (Sophus order);
// pose increments are right-multiplied: T <- T * exp(delta).
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/sim3.hpp>

#include "mvslam/api/types.hpp"

namespace mvslam::core {

using SE3 = Sophus::SE3d;
using SO3 = Sophus::SO3d;
using Sim3 = Sophus::Sim3d;
using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;
using Mat4 = Eigen::Matrix4d;
using Mat6 = Eigen::Matrix<double, 6, 1 * 6>;
using MatX = Eigen::MatrixXd;
using VecX = Eigen::VectorXd;
using RowMatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

inline SE3 toSE3(const mvslam::Pose& p) {
  return SE3(Eigen::Quaterniond(p.q.w, p.q.x, p.q.y, p.q.z).normalized(), Vec3(p.t.x, p.t.y, p.t.z));
}
inline mvslam::Pose fromSE3(const SE3& T) {
  const auto q = T.unit_quaternion();
  const auto t = T.translation();
  return mvslam::Pose{{q.w(), q.x(), q.y(), q.z()}, {t.x(), t.y(), t.z()}};
}
inline SE3 toSE3(const std::array<double, 16>& m) {
  Mat4 M;
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) M(r, c) = m[static_cast<std::size_t>(r * 4 + c)];
  return SE3(Eigen::Quaterniond(M.block<3, 3>(0, 0)).normalized(), M.block<3, 1>(0, 3));
}
inline Vec3 toVec3(const mvslam::Vec3& v) { return Vec3(v.x, v.y, v.z); }
inline mvslam::Vec3 fromVec3(const Vec3& v) { return mvslam::Vec3{v.x(), v.y(), v.z()}; }

}  // namespace mvslam::core
