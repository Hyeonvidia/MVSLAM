#pragma once
// Monocular bootstrap: the first frame with enough corners becomes the reference; its tracks are
// followed frame-to-frame (KLT, ids preserved) until the median parallax is sufficient, then a
// two-view reconstruction (H/F selection) yields the relative pose and the first points. The scale
// is normalized so that the median depth equals `mono_init_median_depth`, and the mapper receives a
// complete two-keyframe initial map (KeyframeRequest::initial_map).
#include "frontend/interfaces.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend::cv_impl {

class MonoTwoViewInitializer final : public IInitializer {
 public:
  MonoTwoViewInitializer(const FrontendConfig& c, IFeatureDetector& detector, IFeatureTracker& tracker,
                         const sensors::CameraRig& rig, map::MapStore& maps)
      : c_(c), detector_(detector), tracker_(tracker), rig_(rig), maps_(maps) {}
  bool tryInitialize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out) override;
  void reset() override { has_ref_ = false; }

 private:
  bool startReference(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input);

  FrontendConfig c_;
  IFeatureDetector& detector_;
  IFeatureTracker& tracker_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  bool has_ref_ = false;
  int frames_since_ref_ = 0;
  Frame ref_, prev_;
  std::shared_ptr<ImageState> prev_image_;
  core::ImageBuffer ref_raw_;
};

}  // namespace mvslam::frontend::cv_impl
