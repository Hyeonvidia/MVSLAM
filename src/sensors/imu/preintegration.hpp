#pragma once
// IMU preintegration between two body states (Forster et al. 2017 as implemented by ORB-SLAM3
// IMU::Preintegrated, in double precision): dR, dV, dP, bias Jacobians, 9x9 covariance and the 6x6
// bias random-walk covariance. Measurements are kept so the integral can be redone with a new bias
// (ORB-SLAM3 Reintegrate) or merged with a predecessor (keyframe culling). Lives in sensors/ so the
// map layer can hold it without depending on the backend. docs/design/03-backend.md §4.
#include <memory>
#include <vector>

#include "core/math/se3.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::sensors::imu {

struct Measurement { core::Vec3 acc, gyro; double dt; };

struct Preintegration {
  core::Vec6 bias_lin = core::Vec6::Zero();           // [bg, ba] the integral was computed with
  core::Mat3 dR = core::Mat3::Identity();
  core::Vec3 dV = core::Vec3::Zero(), dP = core::Vec3::Zero();
  core::Mat3 JRg = core::Mat3::Zero(), JVg = core::Mat3::Zero(), JVa = core::Mat3::Zero(), JPg = core::Mat3::Zero(), JPa = core::Mat3::Zero();
  Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();      // [dR, dV, dP]
  Eigen::Matrix<double, 6, 6> cov_walk = Eigen::Matrix<double, 6, 6>::Zero(); // bias random walk over dt
  double dt = 0;
  std::vector<Measurement> measurements;

  // First-order bias-corrected deltas (ORB-SLAM3 GetDeltaRotation/Velocity/Position).
  core::Mat3 deltaR(const core::Vec6& bias) const;
  core::Vec3 deltaV(const core::Vec6& bias) const;
  core::Vec3 deltaP(const core::Vec6& bias) const;
  // 9x9 information (symmetrized inverse covariance, eigenvalues floored) and its Cholesky factor
  // transposed (whitening matrix W such that W^T W = information).
  Eigen::Matrix<double, 9, 9> information() const;
  Eigen::Matrix<double, 9, 9> sqrtInformation() const;
  Eigen::Matrix<double, 6, 6> walkSqrtInformation() const;
};

class Preintegrator {
 public:
  explicit Preintegrator(const ImuCalibration& calib);
  void reset(const core::Vec6& bias);
  // One IMU interval (measurement already interpolated/averaged by the caller).
  void integrate(const core::Vec3& acc, const core::Vec3& gyro, double dt);
  void reintegrate(const core::Vec6& new_bias);        // redo with a new linearization bias
  void mergePrevious(const Preintegration& prev);      // prev's measurements first, then ours (keyframe culling)
  std::shared_ptr<const Preintegration> snapshot() const { return std::make_shared<Preintegration>(state_); }
  const Preintegration& state() const { return state_; }
  const ImuCalibration& calibration() const { return calib_; }

 private:
  void integrateInternal(const core::Vec3& acc, const core::Vec3& gyro, double dt);
  ImuCalibration calib_;
  Preintegration state_;
  Eigen::Matrix<double, 6, 6> noise_ = Eigen::Matrix<double, 6, 6>::Zero();       // discrete [gyro, acc] measurement noise
  Eigen::Matrix<double, 6, 6> noise_walk_ = Eigen::Matrix<double, 6, 6>::Zero();  // per-step bias random walk
};

}  // namespace mvslam::sensors::imu
