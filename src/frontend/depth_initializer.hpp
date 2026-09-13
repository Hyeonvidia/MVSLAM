#pragma once
// Stereo / RGB-D bootstrap: the first frame with enough detected features becomes the initial
// keyframe; the mapper turns its stereo/depth measurements into map points (ORB-SLAM3 stereo init).
#include "frontend/interfaces.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend {

class DepthInitializer final : public IInitializer {
 public:
  DepthInitializer(const FrontendConfig& c, SensorMode sensor, IFeatureDetector& detector, const sensors::CameraRig& rig, map::MapStore& maps)
      : c_(c), sensor_(sensor), detector_(detector), rig_(rig), maps_(maps) {}
  bool tryInitialize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out) override;
  void reset() override {}
 private:
  FrontendConfig c_;
  SensorMode sensor_;
  IFeatureDetector& detector_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
};

class NullInitializer final : public IInitializer {  // placeholder until the mono two-view initializer lands
 public:
  bool tryInitialize(Frame&, const std::shared_ptr<ImageState>&, const FrameInput&, KeyframeRequest&) override { return false; }
  void reset() override {}
};

}  // namespace mvslam::frontend
