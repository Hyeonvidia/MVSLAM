#pragma once
// KLT tracker (OV2SLAM lineage): 3D tracks start from the projected prediction on 2 levels,
// 2D tracks and 3D failures retry on the full pyramid; forward-backward check on level 0 only.
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend::cv_impl {

class KltTracker final : public IFeatureTracker {
 public:
  KltTracker(const FrontendConfig& c, const sensors::CameraRig& rig) : c_(c), rig_(rig) {}
  Stats track(const ImageState& prev, const ImageState& cur, const Frame& prev_frame, Frame& cur_frame,
              const core::SE3& predicted_T_wb) override;
 private:
  FrontendConfig c_;
  const sensors::CameraRig& rig_;
};

}  // namespace mvslam::frontend::cv_impl
