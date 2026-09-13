#pragma once
// IMU initialization state machine (ORB-SLAM3 InitializeIMU / VIBA1 / VIBA2 / ScaleRefinement with
// the DM-VIO scale-variance criterion as a diagnostic). Runs on the optimizer worker after each
// keyframe: inertial-only MAP with poses fixed -> scaled rotation of the map (gravity to -z, mono
// scale) -> visual-inertial BA with staged bias priors. docs/design/03-backend.md §4.
#include <memory>

#include "backend/local_ba/inertial_bundle_adjuster.hpp"
#include "backend/solvers/solver.hpp"
#include "map/map.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend::imu {

enum class ImuInitState { Uninitialized, Initialized, Refine1, Refine2, Done };

struct ImuInitStep {
  ImuInitState before = ImuInitState::Uninitialized, after = ImuInitState::Uninitialized;
  bool applied_scaled_rotation = false;
  double scale = 1.0, scale_variance = 0;
  core::SO3 R_gw;   // rotation applied to the map (identity when nothing was applied)
};

class ImuInitializer {
 public:
  ImuInitializer(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver);
  ImuInitState state() const { return state_; }
  // Advances the state machine for the map holding `latest_kf`. Rewrites the map (velocities, biases,
  // scale, gravity alignment) under its write lock and runs the visual-inertial BA when a stage completes.
  ImuInitStep step(map::Map& map, core::KeyframeId latest_kf);
  void reset() { state_ = ImuInitState::Uninitialized; }

 private:
  bool runStage(map::Map& map, double prior_gyro, double prior_acc, bool scale_free, int ba_iterations, ImuInitStep& step);
  Config cfg_;
  const sensors::CameraRig& rig_;
  std::shared_ptr<graph::ISolver> solver_;
  InertialBundleAdjuster ba_;
  ImuInitState state_ = ImuInitState::Uninitialized;
};

}  // namespace mvslam::backend::imu
