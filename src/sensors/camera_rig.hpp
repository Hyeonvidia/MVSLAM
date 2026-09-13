#pragma once
// N-camera rig with one body frame (docs/design/00-overview.md §3 principle 2). Every backend pose
// is T_wb; camera i sees the world through T_cw = T_bc[i]^{-1} * T_wb^{-1}.
#include <memory>
#include <vector>

#include "core/math/se3.hpp"
#include "sensors/camera_model.hpp"

namespace mvslam::sensors {

struct ImuCalibration {
  double rate_hz = 200, noise_gyro = 0, noise_acc = 0, walk_gyro = 0, walk_acc = 0, gravity = 9.81;
  core::SE3 T_bi;  // imu -> body (identity when body == imu)
};

class CameraRig {
 public:
  explicit CameraRig(const RigConfig& cfg);
  int numCameras() const { return static_cast<int>(cameras_.size()); }
  const ICameraModel& camera(int i) const { return *cameras_.at(static_cast<std::size_t>(i)); }
  const core::SE3& T_bc(int i) const { return T_bc_.at(static_cast<std::size_t>(i)); }
  const core::SE3& T_cb(int i) const { return T_cb_.at(static_cast<std::size_t>(i)); }
  double baseline() const { return baseline_; }  // 0 for mono / rgbd
  bool hasImu() const { return imu_.has_value(); }
  const ImuCalibration& imu() const { return *imu_; }

 private:
  std::vector<std::unique_ptr<ICameraModel>> cameras_;
  std::vector<core::SE3> T_bc_, T_cb_;
  double baseline_ = 0;
  std::optional<ImuCalibration> imu_;
};

}  // namespace mvslam::sensors
