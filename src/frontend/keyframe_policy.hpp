#pragma once
// OV2SLAM parallax/track-ratio rule + decorators for close points (stereo/rgbd) and IMU timing.
#include <memory>

#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::frontend {

class ParallaxTrackRatioPolicy final : public IKeyframePolicy {
 public:
  explicit ParallaxTrackRatioPolicy(const FrontendConfig& c) : c_(c) {}
  bool needKeyframe(const Context& ctx) const override;
 private:
  FrontendConfig c_;
};

// Decorator: adds ORB-SLAM3 close-point rule and a stereo/rgbd time limit.
class ClosePointRulePolicy final : public IKeyframePolicy {
 public:
  ClosePointRulePolicy(std::unique_ptr<IKeyframePolicy> inner, const FrontendConfig& c) : inner_(std::move(inner)), c_(c) {}
  bool needKeyframe(const Context& ctx) const override;
 private:
  std::unique_ptr<IKeyframePolicy> inner_;
  FrontendConfig c_;
};

// Decorator: IMU timing rules (0.5 s after init, 0.25 s before init).
class InertialTimeRulePolicy final : public IKeyframePolicy {
 public:
  InertialTimeRulePolicy(std::unique_ptr<IKeyframePolicy> inner, const FrontendConfig& c) : inner_(std::move(inner)), c_(c) {}
  bool needKeyframe(const Context& ctx) const override;
 private:
  std::unique_ptr<IKeyframePolicy> inner_;
  FrontendConfig c_;
};

}  // namespace mvslam::frontend
