#pragma once
// IMU preintegration between two body states (ORB-SLAM3 formulation in double precision):
// dR, dV, dP, 9x9 covariance, bias Jacobians (JRg JVg JVa JPg JPa), raw measurements kept for
// re-integration and serialization. docs/design/03-backend.md §4. Implementation: P2.
#include <memory>
#include <vector>

#include "core/math/se3.hpp"
#include "mvslam/api/types.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend::imu {

struct Preintegration {
  core::Vec6 bias_lin = core::Vec6::Zero();           // [bg, ba] used for integration
  core::Mat3 dR = core::Mat3::Identity();
  core::Vec3 dV = core::Vec3::Zero(), dP = core::Vec3::Zero();
  core::Mat3 JRg = core::Mat3::Zero(), JVg = core::Mat3::Zero(), JVa = core::Mat3::Zero(), JPg = core::Mat3::Zero(), JPa = core::Mat3::Zero();
  Eigen::Matrix<double, 9, 9> cov = Eigen::Matrix<double, 9, 9>::Zero();
  Eigen::Matrix<double, 6, 6> cov_walk = Eigen::Matrix<double, 6, 6>::Zero();
  double dt = 0;
  std::vector<ImuSample> raw;                         // for reintegrate()/merge()/serialization

  // First-order bias-corrected deltas (ORB GetDeltaRotation/Velocity/Position).
  core::Mat3 deltaR(const core::Vec6& bias) const;
  core::Vec3 deltaV(const core::Vec6& bias) const;
  core::Vec3 deltaP(const core::Vec6& bias) const;
};

class Preintegrator {
 public:
  explicit Preintegrator(const sensors::ImuCalibration& calib, double reintegration_threshold = 0.01);
  void reset(const core::Vec6& bias);
  void integrate(const ImuSample& sample, double dt);
  void reintegrate(const core::Vec6& new_bias);        // when |bias change| > threshold
  void mergePrevious(const Preintegration& prev);      // keyframe culling
  std::shared_ptr<const Preintegration> snapshot() const { return std::make_shared<Preintegration>(state_); }
  const Preintegration& state() const { return state_; }
 private:
  sensors::ImuCalibration calib_;
  double reint_thresh_;
  Preintegration state_;
};

}  // namespace mvslam::backend::imu
