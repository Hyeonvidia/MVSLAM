#pragma once
// Keyframe-time stereo matching by KLT left->right (OV2SLAM stereoMatching): 3-D tracks start from
// their projection into the right camera (1 level), other tracks from the left pixel (full
// pyramid); matches are verified with the rig's epipolar geometry (Sampson distance) and
// triangulated into a depth along the camera-0 ray.
#include "frontend/interfaces.hpp"
#include "frontend/opencv/preprocessor.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend::cv_impl {

class KltStereoMatcher final : public IDepthSource {
 public:
  KltStereoMatcher(const FrontendConfig& c, const sensors::CameraRig& rig);
  std::vector<DepthMeasurement> measure(const KeyframeRequest& req) override;
 private:
  FrontendConfig c_;
  const sensors::CameraRig& rig_;
  OpenCvPreprocessor preprocessor_;
  core::Mat3 F_10_;   // fundamental matrix for undistorted pixels: x1^T F x0 = 0
  core::SE3 T_c1c0_;
  struct Stats { int attempted = 0, rejected_klt = 0, rejected_fb = 0, rejected_epipolar = 0, rejected_depth = 0, rejected_reproj = 0, accepted = 0; } stats_;
};

}  // namespace mvslam::frontend::cv_impl
