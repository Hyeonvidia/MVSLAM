#pragma once
// IMU initialization state machine (State pattern). Transitions are driven by keyframe count/span
// and the marginal variance of the scale (DM-VIO criterion) with ORB-SLAM3 staged priors.
// docs/design/03-backend.md §4. Implementation: P2.
#include <memory>

#include "backend/solvers/solver.hpp"
#include "map/map.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::backend::imu {

enum class ImuInitState { Uninitialized, CoarseInit, Initialized, Refine1, Refine2, Done };

struct ImuInitStep { ImuInitState before, after; bool applied_scaled_rotation = false; double scale = 1.0, scale_variance = 0; };

class ImuInitializer {
 public:
  ImuInitializer(const BackendConfig& cfg, SensorMode sensor, std::shared_ptr<graph::ISolver> solver);
  ImuInitState state() const { return state_; }
  // Called by the optimizer worker after every keyframe; may run inertial-only MAP / VI-BA and
  // rewrite the map (scale, gravity alignment, velocities, biases) under the map write lock.
  ImuInitStep step(map::Map& map, core::KeyframeId latest_kf);
 private:
  BackendConfig cfg_;
  SensorMode sensor_;
  std::shared_ptr<graph::ISolver> solver_;
  ImuInitState state_ = ImuInitState::Uninitialized;
  double t_init_s_ = 0;
};

}  // namespace mvslam::backend::imu
