#include "sensors/camera_model.hpp"

#include <algorithm>
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

KannalaBrandt8Camera::KannalaBrandt8Camera(const CameraConfig& cfg)
    : width_(cfg.width), height_(cfg.height),
      fx_(cfg.intrinsics[0]), fy_(cfg.intrinsics[1]), cx_(cfg.intrinsics[2]), cy_(cfg.intrinsics[3]) {
  if (fx_ <= 0 || fy_ <= 0 || width_ <= 0 || height_ <= 0) throw ConfigError("KannalaBrandt8Camera: invalid intrinsics/size");
  if (cfg.distortion.size() < 4) throw ConfigError("KannalaBrandt8Camera: distortion needs k1 k2 k3 k4");
  for (int i = 0; i < 4; ++i) k_[i] = cfg.distortion[static_cast<std::size_t>(i)];
}

core::Vec2 KannalaBrandt8Camera::project(const core::Vec3& p_c) const {
  const double iz = 1.0 / p_c.z();
  return {fx_ * p_c.x() * iz + cx_, fy_ * p_c.y() * iz + cy_};
}

Eigen::Matrix<double, 2, 3> KannalaBrandt8Camera::projectJacobian(const core::Vec3& p_c) const {
  const double iz = 1.0 / p_c.z(), iz2 = iz * iz;
  Eigen::Matrix<double, 2, 3> J;
  J << fx_ * iz, 0, -fx_ * p_c.x() * iz2,
       0, fy_ * iz, -fy_ * p_c.y() * iz2;
  return J;
}

double KannalaBrandt8Camera::thetaFromDistorted(double theta_d) const {
  double theta = theta_d;  // Newton on f(theta) = theta_d(theta) - theta_d; 10 iterations reach 1e-12 for |theta| < 1.5
  for (int i = 0; i < 10; ++i) {
    const double t2 = theta * theta, t4 = t2 * t2, t6 = t4 * t2, t8 = t4 * t4;
    const double f = theta * (1 + k_[0] * t2 + k_[1] * t4 + k_[2] * t6 + k_[3] * t8) - theta_d;
    const double df = 1 + 3 * k_[0] * t2 + 5 * k_[1] * t4 + 7 * k_[2] * t6 + 9 * k_[3] * t8;
    const double step = f / df;
    theta -= step;
    if (std::abs(step) < 1e-13) break;
  }
  return theta;
}

core::Vec3 KannalaBrandt8Camera::unprojectRaw(const core::Vec2& px_raw) const {
  const double mx = (px_raw.x() - cx_) / fx_, my = (px_raw.y() - cy_) / fy_;
  const double theta_d = std::sqrt(mx * mx + my * my);
  if (theta_d < 1e-9) return {0, 0, 1};
  const double theta = std::min(thetaFromDistorted(theta_d), 1.55);  // rays beyond ~89 deg have no pinhole image
  const double r = std::tan(theta);
  return {mx / theta_d * r, my / theta_d * r, 1.0};
}

core::Vec2 KannalaBrandt8Camera::undistort(const core::Vec2& px_raw) const {
  const core::Vec3 ray = unprojectRaw(px_raw);
  return {fx_ * ray.x() + cx_, fy_ * ray.y() + cy_};
}

core::Vec2 KannalaBrandt8Camera::projectRaw(const core::Vec3& p_c) const {
  const double r = std::sqrt(p_c.x() * p_c.x() + p_c.y() * p_c.y());
  const double theta = std::atan2(r, p_c.z());
  const double t2 = theta * theta, t4 = t2 * t2, t6 = t4 * t2, t8 = t4 * t4;
  const double theta_d = theta * (1 + k_[0] * t2 + k_[1] * t4 + k_[2] * t6 + k_[3] * t8);
  const double scale = r > 1e-12 ? theta_d / r : 1.0;
  return {fx_ * p_c.x() * scale + cx_, fy_ * p_c.y() * scale + cy_};
}

core::Vec2 KannalaBrandt8Camera::undistortToRaw(const core::Vec2& px_undist) const {
  return projectRaw({(px_undist.x() - cx_) / fx_, (px_undist.y() - cy_) / fy_, 1.0});
}

std::unique_ptr<ICameraModel> makeCameraModel(const CameraConfig& cfg) {
  switch (cfg.model) {
    case CameraModelType::Pinhole: return std::make_unique<PinholeCamera>(cfg);
    case CameraModelType::KannalaBrandt8: return std::make_unique<KannalaBrandt8Camera>(cfg);
  }
  throw ConfigError("unknown camera model");
}

}  // namespace mvslam::sensors
