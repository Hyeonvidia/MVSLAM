#pragma once
// Pose-inertial frame optimization (ORB-SLAM3 PoseInertialOptimizationLastKeyFrame / LastFrame): the
// frame's 15-D state [T_wb, v, b] from its 2-D observations of fixed map points plus an inertial
// constraint. CASE A (map changed since the last frame, or no prior yet): the last keyframe's state is
// constant and the IMU factor spans keyframe -> frame. CASE B: the last frame's state is a variable
// held by the Gaussian prior produced by marginalizing it out of the previous problem (Schur
// complement over the dense 30x30 normal equations, docs/design/03-backend.md §1.4), the IMU factor
// spans frame -> frame. Either way the frame's own information is marginalized into the prior for the
// next frame. Visual factors: Huber -> outlier removal -> L2 refinement as in MotionOnlyBa. Falls back
// to visual-only when no inertial anchor is set.
#include <memory>

#include "backend/graph/factors/prior_factor.hpp"
#include "backend/pose_optimizer.hpp"
#include "backend/solvers/solver.hpp"
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class PoseInertialOptimizer final : public frontend::IPoseOptimizer {
 public:
  PoseInertialOptimizer(const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver, const FrontendConfig& fcfg, const BackendConfig& bcfg)
      : rig_(rig), solver_(std::move(solver)), f_(fcfg), b_(bcfg), visual_(rig, solver_, fcfg, bcfg) {}
  frontend::PoseOptimizationResult optimize(frontend::Frame& frame, double max_time_ms) override;
  void setInertialAnchor(const frontend::InertialAnchor* anchor) override { anchor_ = anchor; }
  bool hasPrior() const { return prior_ != nullptr; }

 private:
  const sensors::CameraRig& rig_;
  std::shared_ptr<graph::ISolver> solver_;
  FrontendConfig f_;
  BackendConfig b_;
  MotionOnlyBa visual_;
  const frontend::InertialAnchor* anchor_ = nullptr;
  std::unique_ptr<graph::PriorFactor> prior_;   // over the last frame's [pose, vel, bias] (keys 1, 2, 3)
};

}  // namespace mvslam::backend
