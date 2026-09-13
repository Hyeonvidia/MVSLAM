#include "frontend/visual_frontend.hpp"

#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::frontend {

VisualFrontend::VisualFrontend(FrontendConfig cfg, bool inertial, FrontendComponents c)
    : cfg_(cfg), c_(std::move(c)),
      sm_({cfg.recently_lost_inertial_s, cfg.recently_lost_visual_s, cfg.new_map_min_keyframes}, inertial) {}

TrackResult VisualFrontend::process(const FrameInput& input, std::optional<KeyframeRequest>& kf_request) {
  core::ScopedTimer timer("frontend.process");
  TrackResult result;
  result.timestamp = input.timestamp;
  if (input.images.empty()) { result.state = sm_.state(); return result; }

  // 1. preprocess (pyramid); 2. predict; 3. track; 4. filter; 5. optimize; 6. state; 7. keyframe.
  auto image = c_.preprocessor ? c_.preprocessor->process(input.images[0]) : nullptr;
  Frame cur;
  cur.timestamp = input.timestamp;
  if (has_last_ && image && last_image_ && c_.tracker) {
    const core::SE3 predicted = c_.predictor ? c_.predictor->predict(last_, input.timestamp) : last_.T_wb;
    cur.T_wb = predicted;
    c_.tracker->track(*last_image_, *image, last_, cur, predicted);
    if (c_.outlier_filter) c_.outlier_filter->filter(last_, cur);
    PoseOptimizationResult po;
    if (c_.pose_optimizer) po = c_.pose_optimizer->optimize(cur, cfg_.pose_max_time_ms);
    if (c_.predictor) c_.predictor->update(last_, cur);
    result.stats.inliers = po.inliers;
    (void)sm_.onFrame(input.timestamp, po.ok, 0);
  }
  // Skeleton: initialization / relocalization / keyframe decision are wired in P0 (see docs/design/09-roadmap.md).
  ++frames_since_kf_;
  last_ = cur;
  last_image_ = std::move(image);
  has_last_ = true;
  result.state = sm_.state();
  result.stats.tracks = static_cast<int>(cur.tracks.size());
  kf_request.reset();
  return result;
}

}  // namespace mvslam::frontend
