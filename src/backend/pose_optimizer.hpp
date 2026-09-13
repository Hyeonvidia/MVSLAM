#pragma once
// Motion-only BA (frontend::IPoseOptimizer implementation): one body pose, all 3D tracks as
// constant points, Huber -> outlier removal -> optional L2 refinement (OV2SLAM 2-pass).
#include <memory>

#include "backend/solvers/solver.hpp"
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class MotionOnlyBa final : public frontend::IPoseOptimizer {
 public:
  MotionOnlyBa(const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver, const FrontendConfig& fcfg,
               const BackendConfig& bcfg)
      : rig_(rig), solver_(std::move(solver)), f_(fcfg), b_(bcfg) {}
  frontend::PoseOptimizationResult optimize(frontend::Frame& frame, double max_time_ms) override;
  // Marks outliers in `frame.tracks` (is3d=false) — kept separate so callers can inspect first.
  std::vector<int> lastOutliers() const { return outliers_; }
 private:
  const sensors::CameraRig& rig_;
  std::shared_ptr<graph::ISolver> solver_;
  FrontendConfig f_;
  BackendConfig b_;
  std::vector<int> outliers_;
};

}  // namespace mvslam::backend
