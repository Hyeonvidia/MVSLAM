#pragma once
// IMU-based pose prediction (docs/design/02-frontend.md §2.9). Before the IMU is initialized only the
// gyroscope is trusted: rotation from the preintegrated dR (zero bias) and translation from the
// constant-velocity model. After initialization the full preintegration is propagated from the last
// frame state (pose, velocity, bias) with gravity along -z (ORB-SLAM3 PredictStateIMU).
#include "frontend/interfaces.hpp"
#include "frontend/pose_predictor.hpp"
#include "sensors/imu/preintegration.hpp"

namespace mvslam::frontend {

class ImuPredictor final : public IPosePredictor {
 public:
  struct Prediction { core::SE3 T_wb; core::Vec3 v_w; };
  // Called by the frontend before predict(): the integral since `last`, whether gravity is known,
  // and gravity magnitude.
  void setPreintegration(const sensors::imu::Preintegration* p, bool imu_initialized, double gravity) {
    preint_ = p; initialized_ = imu_initialized; g_w_ = core::Vec3(0, 0, -gravity);
  }
  Prediction predictState(const Frame& last, Timestamp t_next) const {
    Prediction pr{last.T_wb, last.v_w};
    if (!preint_ || preint_->dt <= 0) { pr.T_wb = cv_.predict(last, t_next); return pr; }
    const core::Mat3 Ri = last.T_wb.rotationMatrix();
    if (!initialized_) {  // gyro-only attitude, constant-velocity translation
      const core::SE3 cv = cv_.predict(last, t_next);
      pr.T_wb = core::SE3(core::SO3(Eigen::Quaterniond(Ri * preint_->deltaR(core::Vec6::Zero()))), cv.translation());
      return pr;
    }
    const double dt = preint_->dt;
    const core::Mat3 Rj = Ri * preint_->deltaR(last.bias);
    pr.v_w = last.v_w + g_w_ * dt + Ri * preint_->deltaV(last.bias);
    const core::Vec3 pj = last.T_wb.translation() + last.v_w * dt + 0.5 * g_w_ * dt * dt + Ri * preint_->deltaP(last.bias);
    pr.T_wb = core::SE3(core::SO3(Eigen::Quaterniond(Rj)), pj);
    return pr;
  }
  core::SE3 predict(const Frame& last, Timestamp t_next) override { return predictState(last, t_next).T_wb; }
  void update(const Frame& last, const Frame& cur) override { cv_.update(last, cur); }
  void reset() override { cv_.reset(); preint_ = nullptr; }

 private:
  mutable ConstantVelocityPredictor cv_;
  const sensors::imu::Preintegration* preint_ = nullptr;
  bool initialized_ = false;
  core::Vec3 g_w_ = core::Vec3(0, 0, -9.81);
};

}  // namespace mvslam::frontend
