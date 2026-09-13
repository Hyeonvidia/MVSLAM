#pragma once
// Visual-inertial BA over a contiguous keyframe chain (ORB-SLAM3 LocalInertialBA / FullInertialBA
// with bInit): poses, velocities and biases of the window keyframes, their map points, reprojection
// factors (Huber) from window + fixed observers, ImuFactor and BiasRandomWalkFactor between
// consecutive keyframes (the keyframe before the window is the fixed inertial boundary), optional
// bias priors (initialization stages). docs/design/03-backend.md §2, §4.
#include <memory>
#include <stop_token>
#include <vector>

#include "backend/solvers/solver.hpp"
#include "map/map.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

struct InertialBaOptions {
  int iterations = 10;
  double bias_prior_gyro = 0, bias_prior_acc = 0;  // > 0: BiasPriorFactor on every window keyframe bias
  double max_time_s = 0;                            // 0 = no wall-clock limit
  int max_fixed_visual = 200;                       // observers outside the window kept as fixed poses
  double boundary_info_scale = 1e-2;                // information scale of the IMU edge to the fixed boundary keyframe (ORB-SLAM3)
  bool robust_imu = false;                          // Huber(sqrt 16.92) on every IMU edge (ORB-SLAM3 before the second refinement)
};

class InertialBundleAdjuster {
 public:
  InertialBundleAdjuster(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver);
  struct Report { bool ran = false, applied = false, preempted = false; int keyframes = 0, points = 0, observations = 0, imu_factors = 0, outliers = 0; double initial_cost = 0, final_cost = 0; };
  // `window`: consecutive keyframes in prevKf order (oldest first); every entry but possibly the first
  // must carry a preintegration from its predecessor.
  Report run(map::Map& map, const std::vector<core::KeyframeId>& window, const InertialBaOptions& opt, std::stop_token st = {},
             std::function<bool()> newer_keyframe_pending = nullptr);
  const Report& last() const { return last_; }

 private:
  Config cfg_;
  const sensors::CameraRig& rig_;
  std::shared_ptr<graph::ISolver> solver_;
  Report last_;
};

}  // namespace mvslam::backend
