#include "frontend/keyframe_policy.hpp"

#include <algorithm>

namespace mvslam::frontend {

namespace {
int count3d(const Frame* f) {
  if (!f) return 0;
  return static_cast<int>(std::count_if(f->tracks.begin(), f->tracks.end(), [](const Track& t) { return t.is3d; }));
}
}  // namespace

bool ParallaxTrackRatioPolicy::needKeyframe(const Context& ctx) const {
  if (!ctx.cur) return false;
  const int n3d = count3d(ctx.cur);
  const int n3d_kf = std::max(1, count3d(ctx.last_kf));
  const double occupancy = ctx.max_tracks > 0 ? static_cast<double>(ctx.cur->occupied_cells) / ctx.max_tracks : 1.0;
  // Immediate keyframe: starved of features and the optimizer can take it.
  if (occupancy < 0.33 && ctx.frames_since_kf >= 5 && !ctx.optimizer_busy) return true;
  if (n3d < 20 && ctx.frames_since_kf >= 2) return true;
  // Backpressure: healthy tracking and busy backend (or too soon) -> no keyframe.
  if (n3d > 0.5 * ctx.max_tracks && (ctx.optimizer_busy || ctx.frames_since_kf < 2)) return false;
  const bool parallax_gate = ctx.median_parallax_px >= c_.kf_min_parallax_px / 2.0;
  const bool c0 = ctx.median_parallax_px >= c_.kf_min_parallax_px;
  const bool c1 = n3d < c_.kf_track_ratio * n3d_kf;
  const bool c2 = occupancy < c_.kf_occupancy_ratio && n3d < 0.85 * n3d_kf;
  return (c0 || c1 || c2) && parallax_gate;
}

bool ClosePointRulePolicy::needKeyframe(const Context& ctx) const {
  if (ctx.seconds_since_kf > c_.kf_stereo_force_s) return true;
  const bool need_close = ctx.tracked_close < c_.kf_close_tracked_min && ctx.new_close > c_.kf_close_new_min;
  if (need_close && ctx.frames_since_kf >= 2) return true;
  return inner_->needKeyframe(ctx);
}

bool InertialTimeRulePolicy::needKeyframe(const Context& ctx) const {
  const double limit = ctx.imu_initialized ? c_.kf_imu_interval_s : c_.kf_imu_preinit_interval_s;
  if (ctx.seconds_since_kf >= limit) return true;
  return inner_->needKeyframe(ctx);
}

}  // namespace mvslam::frontend
