#pragma once
// 2D-2D outlier filter against the last keyframe: 5-point essential matrix RANSAC on undistorted
// pixels (OV2SLAM epipolar2d2dFiltering). Skipped when parallax is too small (pure rotation) or when
// the inlier set collapses (degenerate estimate).
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend::cv_impl {

class EssentialRansacFilter final : public IOutlierFilter {
 public:
  EssentialRansacFilter(const FrontendConfig& c, const sensors::CameraRig& rig) : c_(c), rig_(rig) {}
  int filter(const Frame& reference, Frame& cur) override;
 private:
  FrontendConfig c_;
  const sensors::CameraRig& rig_;
};

}  // namespace mvslam::frontend::cv_impl
