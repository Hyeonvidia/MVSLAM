#pragma once
// Camera model strategy (Strategy pattern). Works on *undistorted normalized* coordinates for
// projection Jacobians (the frontend stores undistorted keypoints), and provides distortion-aware
// pixel <-> ray conversions for input handling. Thread-safe: immutable after construction.
#include <memory>
#include <string>

#include "core/math/se3.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::sensors {

class ICameraModel {
 public:
  virtual ~ICameraModel() = default;
  virtual CameraModelType type() const = 0;
  virtual int width() const = 0;
  virtual int height() const = 0;
  // Ideal (undistorted) projection of a camera-frame point; also its 2x3 Jacobian w.r.t. the point.
  virtual core::Vec2 project(const core::Vec3& p_c) const = 0;
  virtual Eigen::Matrix<double, 2, 3> projectJacobian(const core::Vec3& p_c) const = 0;
  // Pixel (raw, distorted) -> unit-depth ray in camera frame (undistorted).
  virtual core::Vec3 unprojectRaw(const core::Vec2& px_raw) const = 0;
  // Raw pixel -> undistorted pixel (what the frontend stores), and the inverse mapping.
  virtual core::Vec2 undistort(const core::Vec2& px_raw) const = 0;
  virtual core::Vec2 undistortToRaw(const core::Vec2& px_undist) const = 0;
  // Undistorted pixel -> unit-depth ray (ideal pinhole inverse).
  core::Vec3 unprojectUndistorted(const core::Vec2& px) const { return {(px.x() - cx()) / fx(), (px.y() - cy()) / fy(), 1.0}; }
  virtual double cx() const = 0;
  virtual double cy() const = 0;
  virtual double fx() const = 0;
  virtual double fy() const = 0;
  virtual bool inImage(const core::Vec2& px, double margin = 0) const {
    return px.x() >= margin && px.y() >= margin && px.x() < width() - margin && px.y() < height() - margin;
  }
};

// Pinhole + radial-tangential (k1 k2 p1 p2 [k3]).
class PinholeCamera final : public ICameraModel {
 public:
  explicit PinholeCamera(const CameraConfig& cfg);
  CameraModelType type() const override { return CameraModelType::Pinhole; }
  int width() const override { return width_; }
  int height() const override { return height_; }
  core::Vec2 project(const core::Vec3& p_c) const override;
  Eigen::Matrix<double, 2, 3> projectJacobian(const core::Vec3& p_c) const override;
  core::Vec3 unprojectRaw(const core::Vec2& px_raw) const override;
  core::Vec2 undistort(const core::Vec2& px_raw) const override;
  core::Vec2 undistortToRaw(const core::Vec2& px_undist) const override;
  double fx() const override { return fx_; }
  double fy() const override { return fy_; }
  double cx() const override { return cx_; }
  double cy() const override { return cy_; }

 private:
  core::Vec2 distortNormalized(const core::Vec2& n) const;
  int width_, height_;
  double fx_, fy_, cx_, cy_;
  double k1_ = 0, k2_ = 0, p1_ = 0, p2_ = 0, k3_ = 0;
  bool has_distortion_ = false;
};

// Factory: CameraConfig -> concrete model (throws ConfigError for unsupported types).
std::unique_ptr<ICameraModel> makeCameraModel(const CameraConfig& cfg);

}  // namespace mvslam::sensors
