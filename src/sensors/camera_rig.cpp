#include "sensors/camera_rig.hpp"

#include "mvslam/api/errors.hpp"

namespace mvslam::sensors {

CameraRig::CameraRig(const RigConfig& cfg) {
  if (cfg.cameras.empty()) throw ConfigError("rig: at least one camera is required");
  for (const auto& c : cfg.cameras) {
    cameras_.push_back(makeCameraModel(c));
    T_bc_.push_back(core::toSE3(c.T_bc));
    T_cb_.push_back(T_bc_.back().inverse());
  }
  if (cfg.cameras.size() >= 2) {
    baseline_ = cfg.stereo_baseline_m > 0 ? cfg.stereo_baseline_m : (T_bc_[0].inverse() * T_bc_[1]).translation().norm();
  }
  if (cfg.imu) {
    ImuCalibration c;
    c.rate_hz = cfg.imu->rate_hz; c.noise_gyro = cfg.imu->noise_gyro; c.noise_acc = cfg.imu->noise_acc;
    c.walk_gyro = cfg.imu->walk_gyro; c.walk_acc = cfg.imu->walk_acc; c.gravity = cfg.imu->gravity;
    c.T_bi = core::toSE3(cfg.imu->T_bi);
    imu_ = c;
  }
}

}  // namespace mvslam::sensors
