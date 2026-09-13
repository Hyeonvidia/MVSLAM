#include "frontend/opencv/mono_initializer.hpp"

#include <algorithm>
#include <unordered_map>

#include "core/log.hpp"
#include "frontend/opencv/two_view_reconstruction.hpp"

namespace mvslam::frontend::cv_impl {

namespace {
constexpr int kMaxFramesSinceReference = 60;  // stale reference: restart from the current frame
}

bool MonoTwoViewInitializer::startReference(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input) {
  const auto& cam = rig_.camera(0);
  const auto pts = detector_.detect(*image, {}, detector_.cellCount(cam.width(), cam.height()));
  has_ref_ = false;
  if (static_cast<int>(pts.size()) < c_.init_min_features) return false;
  cur.tracks.clear();
  for (const auto& p : pts) {
    Track t;
    t.id = maps_.trackIds().next();
    t.uv_raw = p;
    t.uv = cam.undistort(p);
    cur.tracks.push_back(t);
  }
  ref_ = cur;
  prev_ = cur;
  prev_image_ = image;
  ref_raw_ = core::ImageBuffer::copy(input.images[0]);
  has_ref_ = true;
  frames_since_ref_ = 0;
  return true;
}

bool MonoTwoViewInitializer::tryInitialize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out) {
  if (!has_ref_) return startReference(cur, image, input), false;
  ++frames_since_ref_;
  tracker_.track(*prev_image_, *image, prev_, cur, prev_.T_wb);
  cur.T_wb = ref_.T_wb;
  if (static_cast<int>(cur.tracks.size()) < c_.init_min_features || frames_since_ref_ > kMaxFramesSinceReference) {
    startReference(cur, image, input);
    return false;
  }
  prev_ = cur;
  prev_image_ = image;

  std::unordered_map<core::TrackId, core::Vec2> ref_uv;
  for (const auto& t : ref_.tracks) ref_uv.emplace(t.id, t.uv);
  std::vector<TwoViewMatch> matches;
  std::vector<std::size_t> cur_index;
  std::vector<double> disp;
  for (std::size_t i = 0; i < cur.tracks.size(); ++i) {
    auto it = ref_uv.find(cur.tracks[i].id);
    if (it == ref_uv.end()) continue;
    matches.push_back({cur.tracks[i].id, it->second, cur.tracks[i].uv});
    cur_index.push_back(i);
    disp.push_back((cur.tracks[i].uv - it->second).norm());
  }
  if (static_cast<int>(matches.size()) < c_.init_min_features) return false;
  std::nth_element(disp.begin(), disp.begin() + static_cast<long>(disp.size() / 2), disp.end());
  if (disp[disp.size() / 2] < c_.mono_init_min_parallax_px) return false;

  TwoViewOptions opt;
  opt.ransac_iters = c_.ransac_iters * 2;
  auto tv = reconstructTwoView(matches, rig_.camera(0), opt);
  if (!tv) return false;

  // Scale normalization: median depth in the reference camera -> mono_init_median_depth.
  std::vector<double> depths;
  for (std::size_t i = 0; i < matches.size(); ++i) if (tv->triangulated[i]) depths.push_back(tv->X_c1[i].z());
  std::nth_element(depths.begin(), depths.begin() + static_cast<long>(depths.size() / 2), depths.end());
  const double scale = c_.mono_init_median_depth / depths[depths.size() / 2];
  const core::SE3 T_c2c1(tv->T_c2c1.so3(), tv->T_c2c1.translation() * scale);
  const core::SE3 T_wc1 = ref_.T_wb * rig_.T_bc(0);
  cur.T_wb = T_wc1 * T_c2c1.inverse() * rig_.T_cb(0);

  KeyframeRequest::InitialMap init;
  init.first_kf = maps_.keyframeIds().next();
  init.first_frame = ref_;
  init.first_frame.reference_kf = init.first_kf;
  init.first_left_raw = ref_raw_;
  for (std::size_t i = 0; i < matches.size(); ++i) {
    if (!tv->triangulated[i]) continue;
    const core::Vec3 X_w = T_wc1 * (tv->X_c1[i] * scale);
    init.points.emplace_back(matches[i].id, X_w);
    auto& t = cur.tracks[cur_index[i]];
    t.is3d = true;
    t.X_w = X_w;
  }
  MVSLAM_INFO("mono init: {} points from {} matches (R_H={:.2f}, model={}, {} frames after the reference)", init.points.size(),
              matches.size(), tv->r_h, tv->used_homography ? "H" : "F", frames_since_ref_);
  out.initial_map = std::move(init);
  out.kf_id = maps_.keyframeIds().next();
  out.initial = true;
  out.image_state = image;
  out.left_raw = core::ImageBuffer::copy(input.images[0]);
  cur.reference_kf = out.kf_id;
  cur.T_kb = core::SE3();
  out.frame = cur;
  has_ref_ = false;
  return true;
}

}  // namespace mvslam::frontend::cv_impl
