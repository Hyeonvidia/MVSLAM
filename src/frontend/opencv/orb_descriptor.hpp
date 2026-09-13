#pragma once
// ORB descriptors at keyframe time (single scale). Keypoints filtered away by OpenCV (image border)
// get an all-zero descriptor so indices stay aligned with the keypoint list.
#include <opencv2/features2d.hpp>

#include "frontend/interfaces.hpp"

namespace mvslam::frontend::cv_impl {

class OrbDescriptorExtractor final : public IDescriptorExtractor {
 public:
  OrbDescriptorExtractor() : orb_(cv::ORB::create(2000, 1.2f, 1, 19, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20)) {}
  std::vector<std::uint8_t> describe(const ImageView& raw, const std::vector<core::Vec2>& uv_raw) override;
  int stride() const override { return 32; }
 private:
  cv::Ptr<cv::ORB> orb_;
};

}  // namespace mvslam::frontend::cv_impl
