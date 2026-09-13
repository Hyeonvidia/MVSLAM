#include "sensors/camera_model.hpp"

#include <cmath>

#include "mvslam/api/errors.hpp"

namespace mvslam::sensors {

PinholeCamera::PinholeCamera(const CameraConfig& cfg)
    : width_(cfg.width), height_(cfg.height),
      fx_(cfg.intrinsics[0]), fy_(cfg.intrinsics[1]), cx_(cfg.intrinsics[2]), cy_(cfg.intrinsics[3]) {
  if (fx_ <= 0 || fy_ <= 0 || width_ <= 0 || height_ <= 0) throw ConfigError("PinholeCamera: invalid intrinsics/size");
  const auto& d = cfg.distortion;
  if (!d.empty()) {
    if (d.size() < 4) throw ConfigError("PinholeCamera: distortion needs k1 k2 p1 p2 [k3]");
    k1_ = d[0]; k2_ = d[1]; p1_ = d[2]; p2_ = d[3];
    if (d.size() > 4) k3_ = d[4];
    has_distortion_ = (k1_ != 0 || k2_ != 0 || p1_ != 0 || p2_ != 0 || k3_ != 0);
  }
}

core::Vec2 PinholeCamera::project(const core::Vec3& p_c) const {
  const double iz = 1.0 / p_c.z();
  return {fx_ * p_c.x() * iz + cx_, fy_ * p_c.y() * iz + cy_};
}

Eigen::Matrix<double, 2, 3> PinholeCamera::projectJacobian(const core::Vec3& p_c) const {
  const double iz = 1.0 / p_c.z(), iz2 = iz * iz;
  Eigen::Matrix<double, 2, 3> J;
  J << fx_ * iz, 0, -fx_ * p_c.x() * iz2,
       0, fy_ * iz, -fy_ * p_c.y() * iz2;
  return J;
}

core::Vec2 PinholeCamera::distortNormalized(const core::Vec2& n) const {
  const double x = n.x(), y = n.y(), r2 = x * x + y * y;
  const double radial = 1 + k1_ * r2 + k2_ * r2 * r2 + k3_ * r2 * r2 * r2;
  return {x * radial + 2 * p1_ * x * y + p2_ * (r2 + 2 * x * x),
          y * radial + p1_ * (r2 + 2 * y * y) + 2 * p2_ * x * y};
}

core::Vec3 PinholeCamera::unprojectRaw(const core::Vec2& px_raw) const {
  core::Vec2 n((px_raw.x() - cx_) / fx_, (px_raw.y() - cy_) / fy_);
  if (has_distortion_) {
    // Fixed-point undistortion (OpenCV-style), 10 iterations is plenty for moderate distortion.
    core::Vec2 u = n;
    for (int i = 0; i < 10; ++i) {
      const core::Vec2 d = distortNormalized(u);
      u += n - d;
    }
    n = u;
  }
  return {n.x(), n.y(), 1.0};
}

core::Vec2 PinholeCamera::undistort(const core::Vec2& px_raw) const {
  const core::Vec3 r = unprojectRaw(px_raw);
  return {fx_ * r.x() + cx_, fy_ * r.y() + cy_};
}

core::Vec2 PinholeCamera::undistortToRaw(const core::Vec2& px_undist) const {
  const core::Vec2 n((px_undist.x() - cx_) / fx_, (px_undist.y() - cy_) / fy_);
  const core::Vec2 d = has_distortion_ ? distortNormalized(n) : n;
  return {fx_ * d.x() + cx_, fy_ * d.y() + cy_};
}

std::unique_ptr<ICameraModel> makeCameraModel(const CameraConfig& cfg) {
  switch (cfg.model) {
    case CameraModelType::Pinhole: return std::make_unique<PinholeCamera>(cfg);
    case CameraModelType::KannalaBrandt8: throw ConfigError("KannalaBrandt8 camera model: not implemented yet (P1)");
  }
  throw ConfigError("unknown camera model");
}

}  // namespace mvslam::sensors
