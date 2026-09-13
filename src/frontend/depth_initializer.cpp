#include "frontend/depth_initializer.hpp"

namespace mvslam::frontend {

bool DepthInitializer::tryInitialize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out) {
  const auto& cam = rig_.camera(0);
  const int budget = detector_.cellCount(cam.width(), cam.height());
  const auto pts = detector_.detect(*image, {}, budget);
  if (static_cast<int>(pts.size()) < c_.init_min_features) return false;
  if (sensor_ == SensorMode::Stereo && input.images.size() < 2) return false;
  if (sensor_ == SensorMode::RgbD && !input.depth) return false;
  cur.tracks.clear();  // cur.T_wb was seeded by the caller (identity for the first map)
  for (const auto& p : pts) {
    Track t;
    t.id = maps_.trackIds().next();
    t.uv_raw = p;
    t.uv = cam.undistort(p);
    cur.tracks.push_back(t);
  }
  out.kf_id = maps_.keyframeIds().next();
  out.initial = true;
  out.image_state = image;
  out.left_raw = core::ImageBuffer::copy(input.images[0]);
  if (sensor_ == SensorMode::Stereo) out.right_raw = core::ImageBuffer::copy(input.images[1]);
  if (sensor_ == SensorMode::RgbD) out.depth = core::ImageBuffer::copy(*input.depth);
  cur.reference_kf = out.kf_id;
  cur.T_kb = core::SE3();
  out.frame = cur;
  return true;
}

}  // namespace mvslam::frontend
