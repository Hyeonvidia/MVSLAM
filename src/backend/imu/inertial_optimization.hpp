#pragma once
// Inertial-only MAP over a chain of keyframes with fixed poses (ORB-SLAM3 InertialOptimization):
// velocities, one shared bias, gravity direction and (mono) log-scale from the preintegrations
// between consecutive keyframes. Used by the IMU initializer (staged priors) and by the mono scale
// refinement (only gravity + scale free). docs/design/03-backend.md §4.
#include <memory>
#include <unordered_map>
#include <vector>

#include "backend/solvers/solver.hpp"
#include "core/ids.hpp"
#include "core/math/se3.hpp"
#include "backend/imu/preintegrator.hpp"

namespace mvslam::backend::imu {

struct KeyframeInertialState {
  core::KeyframeId id;
  std::int64_t timestamp = 0;
  core::SE3 T_wb;
  core::Vec3 v_w = core::Vec3::Zero();
  core::Vec6 bias = core::Vec6::Zero();
  std::shared_ptr<const Preintegration> preint;  // from the previous entry of the chain (null for the first)
};

struct InertialOnlyOptions {
  double prior_gyro = 1e2, prior_acc = 1e10;   // staged bias priors (ORB-SLAM3: 1e2/1e10 -> 1/1e5 -> 0/0)
  bool optimize_scale = true;                  // mono only; stereo/rgbd keep s = 1
  bool optimize_gravity = true;
  bool optimize_velocities = true, optimize_bias = true;
  int iterations = 200;
  double gravity = 9.81;
};

struct InertialOnlyResult {
  bool ok = false;
  double scale = 1.0, scale_variance = 0;      // variance of log-scale from the dense Hessian
  core::SO3 R_wg;                              // gravity in the map frame: g = R_wg [0 0 -G]
  core::Vec6 bias = core::Vec6::Zero();
  std::vector<core::Vec3> velocities;          // one per chain entry
  double initial_chi2 = 0, final_chi2 = 0;
  int factors = 0;
};

// Initial gravity direction from the accumulated velocity changes (ORB-SLAM3 InitializeIMU dirG).
core::SO3 initialGravityDirection(const std::vector<KeyframeInertialState>& chain);
// Initial velocities from finite differences of the keyframe positions.
std::vector<core::Vec3> initialVelocities(const std::vector<KeyframeInertialState>& chain);

InertialOnlyResult inertialOnlyOptimization(const std::vector<KeyframeInertialState>& chain, const core::SO3& R_wg0, double scale0,
                                            const core::Vec6& bias0, const InertialOnlyOptions& opt, graph::ISolver& solver);

}  // namespace mvslam::backend::imu
