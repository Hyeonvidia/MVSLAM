#pragma once
// P3P-RANSAC (OpenCV AP3P) on the 3-D tracks of a frame; used when KLT collapsed or PnP failed.
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend::cv_impl {

class P3pRansacSolver final : public IRobustPoseSolver {
 public:
  P3pRansacSolver(const FrontendConfig& c, const sensors::CameraRig& rig) : c_(c), rig_(rig) {}
  bool solve(const Frame& frame, core::SE3& T_wb, std::vector<int>& outlier_indices) override;
 private:
  FrontendConfig c_;
  const sensors::CameraRig& rig_;
};

}  // namespace mvslam::frontend::cv_impl
