#include "frontend/visual_frontend.hpp"

#include <unordered_set>

#include "core/log.hpp"
#include "core/profiler.hpp"
#include "frontend/parallax.hpp"

namespace mvslam::frontend {

VisualFrontend::VisualFrontend(FrontendConfig cfg, SensorMode sensor, bool inertial, FrontendComponents c,
                               const sensors::CameraRig& rig, map::MapStore& maps)
    : cfg_(cfg), sensor_(sensor), c_(std::move(c)), rig_(rig), maps_(maps),
      sm_({cfg.recently_lost_inertial_s, cfg.recently_lost_visual_s, cfg.new_map_min_keyframes}, inertial) {
  const auto& cam = rig_.camera(0);
  max_tracks_ = c_.detector ? c_.detector->cellCount(cam.width(), cam.height()) : 300;
}

int VisualFrontend::countOccupiedCells(const Frame& f) const {
  const int cell = std::max(1, cfg_.detector_cell_px);
  const int ncx = std::max(1, rig_.camera(0).width() / cell);
  std::unordered_set<int> cells;
  for (const auto& t : f.tracks) cells.insert(static_cast<int>(t.uv_raw.y()) / cell * ncx + static_cast<int>(t.uv_raw.x()) / cell);
  return static_cast<int>(cells.size());
}

void VisualFrontend::refreshTracksFromMap(Frame& f) {
  // Every frame (not only on map epoch change): per-frame outlier demotion must not stick.
  auto map = maps_.active();
  last_epoch_ = map->changeEpoch();
  map->withReadLock([&](const map::Map& m) {
    for (auto& t : f.tracks) {
      if (m.hasTrackAlias(t.id)) t.id = core::TrackId{m.resolveTrack(t.id).value};  // matched to an existing map point
      auto mp = m.mapPoint(toMapPointId(t.id));
      if (mp && !mp->bad()) { t.is3d = true; t.X_w = mp->position(); }
      else t.is3d = false;
    }
    for (auto& t : last_kf_frame_.tracks) if (m.hasTrackAlias(t.id)) t.id = core::TrackId{m.resolveTrack(t.id).value};
    return 0;
  });
}

bool VisualFrontend::estimatePose(Frame& cur, const IFeatureTracker::Stats& st, TrackResult& r) {
  const int n3d = cur.num3d();
  if (n3d < cfg_.p3p_min_tracks) return false;
  const bool collapsed = st.attempted3d > 0 && st.survived3d < cfg_.p3p_survival_ratio * st.attempted3d;
  if (!c_.pose_optimizer) return false;
  const core::SE3 predicted = cur.T_wb;
  std::vector<bool> was3d(cur.tracks.size());
  for (std::size_t i = 0; i < cur.tracks.size(); ++i) was3d[i] = cur.tracks[i].is3d;
  auto restore3d = [&] { for (std::size_t i = 0; i < cur.tracks.size(); ++i) cur.tracks[i].is3d = was3d[i]; };
  auto p3p = [&]() -> bool {
    if (!c_.robust_pose) return false;
    restore3d();
    core::SE3 T;
    std::vector<int> outliers;
    if (!c_.robust_pose->solve(cur, T, outliers)) return false;
    cur.T_wb = T;
    for (int i : outliers) cur.tracks[static_cast<std::size_t>(i)].is3d = false;
    return true;
  };
  bool p3p_done = false;
  if (p3p_required_ || collapsed) p3p_done = p3p();
  auto po = c_.pose_optimizer->optimize(cur, cfg_.pose_max_time_ms);
  if (!po.ok && !p3p_done) {  // bad prediction: robust re-seed from the 2D-3D matches, then re-optimize
    cur.T_wb = predicted;
    if (p3p()) po = c_.pose_optimizer->optimize(cur, cfg_.pose_max_time_ms);
  }
  r.stats.inliers = po.inliers;
  p3p_required_ = !po.ok;
  if (!po.ok) { cur.T_wb = predicted; restore3d(); }
  return po.ok;
}

void VisualFrontend::createKeyframe(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out) {
  const auto& cam = rig_.camera(0);
  if (c_.detector) {
    std::vector<core::Vec2> existing;
    existing.reserve(cur.tracks.size());
    for (const auto& t : cur.tracks) existing.push_back(t.uv_raw);
    const int budget = std::max(0, max_tracks_ - static_cast<int>(cur.tracks.size()));
    for (const auto& p : c_.detector->detect(*image, existing, budget)) {
      Track t;
      t.id = maps_.trackIds().next();
      t.uv_raw = p;
      t.uv = cam.undistort(p);
      cur.tracks.push_back(t);
    }
  }
  cur.occupied_cells = countOccupiedCells(cur);
  out.kf_id = maps_.keyframeIds().next();
  out.image_state = image;
  out.left_raw = core::ImageBuffer::copy(input.images[0]);
  if (sensor_ == SensorMode::Stereo && input.images.size() > 1) out.right_raw = core::ImageBuffer::copy(input.images[1]);
  if (sensor_ == SensorMode::RgbD && input.depth) out.depth = core::ImageBuffer::copy(*input.depth);
  cur.reference_kf = out.kf_id;
  cur.T_kb = core::SE3();
  out.frame = cur;
  last_kf_frame_ = cur;
  last_kf_id_ = out.kf_id;
  frames_since_kf_ = 0;
}

std::optional<RelocalizationResult> VisualFrontend::localizeOnce(const FrameInput& input) {
  if (!c_.relocalizer || !c_.preprocessor || input.images.empty()) return std::nullopt;
  const std::shared_ptr<ImageState> image = c_.preprocessor->process(input.images[0]);
  return c_.relocalizer->relocalize(*image, input.images[0], maps_.all());
}

bool VisualFrontend::tryRelocalize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, bool all_maps) {
  if (!c_.relocalizer) return false;
  const auto maps = all_maps ? maps_.all() : std::vector<std::shared_ptr<map::Map>>{maps_.active()};
  auto rr = c_.relocalizer->relocalize(*image, input.images[0], maps);
  if (!rr) return false;
  adoptRelocalization(cur, *rr, input.timestamp);
  return true;
}

void VisualFrontend::adoptRelocalization(Frame& cur, const RelocalizationResult& rr, Timestamp t) {
  if (rr.map != maps_.active()->id()) {
    maps_.activate(rr.map);
    MVSLAM_INFO("frontend: switched to map {} by relocalization", rr.map.value);
  }
  cur.T_wb = rr.T_wb;
  cur.tracks = rr.tracks;
  last_kf_id_ = rr.keyframe;
  last_kf_frame_ = Frame{};
  last_kf_frame_.timestamp = t;
  last_kf_frame_.T_wb = rr.T_wk;
  last_kf_frame_.tracks = cur.tracks;
  frames_since_kf_ = 0;
  waiting_frames_ = 0;
  p3p_required_ = false;
  if (c_.predictor) c_.predictor->reset();
  phase_ = Phase::Tracking;
  sm_.onRelocalized(t);
}

void VisualFrontend::restartBootstrap() {
  phase_ = Phase::Bootstrap;
  waiting_frames_ = 0;
  p3p_required_ = false;
  if (c_.initializer) c_.initializer->reset();
  if (c_.predictor) c_.predictor->reset();
  sm_.onNewMap();
}

TrackResult VisualFrontend::process(const FrameInput& input, std::optional<KeyframeRequest>& kf_request) {
  core::ScopedTimer timer("frontend.process");
  TrackResult r;
  r.timestamp = input.timestamp;
  kf_request.reset();
  if (input.images.empty() || !c_.preprocessor) { r.state = sm_.state(); return r; }

  std::shared_ptr<ImageState> image = c_.preprocessor->process(input.images[0]);
  Frame cur;
  cur.id = ++frame_counter_;
  cur.timestamp = input.timestamp;

  if (phase_ == Phase::Bootstrap && reloc_armed_ > 0) {  // MapService::localize(): relocalize into a loaded map first
    if (tryRelocalize(cur, image, input, true)) {
      reloc_armed_ = 0;
      reloc_outcome_ = Result<Pose>::success(core::fromSE3(cur.T_wb));
      r.pose = PoseState{core::fromSE3(cur.T_wb), {}, {}, {}};
      cur.reference_kf = last_kf_id_;
      cur.T_kb = last_kf_frame_.T_wb.inverse() * cur.T_wb;
    } else if (--reloc_armed_ == 0) {
      reloc_outcome_ = Result<Pose>::failure("relocalization failed: no map matched the recent frames");
    }
  } else if (phase_ == Phase::Bootstrap) {
    cur.T_wb = has_last_ ? last_.T_wb : core::SE3();  // a restarted map continues from the last known pose
    KeyframeRequest req;
    if (c_.initializer && c_.initializer->tryInitialize(cur, image, input, req)) {
      last_kf_frame_ = cur;
      last_kf_id_ = req.kf_id;
      frames_since_kf_ = 0;
      waiting_frames_ = 0;
      r.keyframe_id = req.kf_id.value;
      kf_request = std::move(req);
      phase_ = Phase::WaitingFor3d;
    }
  } else {
    refreshTracksFromMap(last_);
    const core::SE3 predicted = c_.predictor ? c_.predictor->predict(last_, input.timestamp) : last_.T_wb;
    cur.T_wb = predicted;
    IFeatureTracker::Stats st;
    if (c_.tracker && last_image_) st = c_.tracker->track(*last_image_, *image, last_, cur, predicted);
    const int before_filter = static_cast<int>(cur.tracks.size());
    int filtered = 0;
    if (c_.outlier_filter && phase_ == Phase::Tracking) filtered = c_.outlier_filter->filter(last_kf_frame_, cur);
    bool ok = estimatePose(cur, st, r);
    lost_frames_ = ok ? 0 : lost_frames_ + 1;
    MVSLAM_DEBUG("frame {}: klt 3d {}/{} 2d {}/{} -> {} tracks, E-filter -{}, 3d {}, inliers {}, ok {}", cur.id, st.survived3d, st.attempted3d,
                 st.survived2d, st.attempted2d, before_filter, filtered, cur.num3d(), r.stats.inliers, ok);

    if (phase_ == Phase::WaitingFor3d) {
      if (ok && cur.num3d() >= cfg_.min_3d_for_ok) {
        phase_ = Phase::Tracking;
        sm_.onInitialized(input.timestamp);
        MVSLAM_INFO("frontend: initialized with {} 3-D tracks", cur.num3d());
      } else if (++waiting_frames_ > cfg_.reinit_after_frames) {
        MVSLAM_WARN("frontend: bootstrap produced no 3-D tracks after {} frames, restarting", waiting_frames_);
        maps_.resetActive();
        restartBootstrap();
      }
    } else {
      const int kfs = static_cast<int>(maps_.active()->withReadLock([](const map::Map& m) { return m.numKeyframes(); }));
      switch (sm_.onFrame(input.timestamp, ok, kfs)) {
        case TrackingStateMachine::Action::StartNewMap: maps_.startNewMap(); restartBootstrap(); break;
        case TrackingStateMachine::Action::ResetMap: maps_.resetActive(); restartBootstrap(); break;
        case TrackingStateMachine::Action::TryRelocalize:
          // A single failed frame is usually recovered by the next frame's P3P re-seed; relocalizing right away
          // replaces the KLT tracks with a smaller, less accurate set (MH01 mono: Sim3 ATE 0.14 -> 0.23 m).
          if ((lost_frames_ >= cfg_.reloc_min_lost_frames || reloc_armed_ > 0) && tryRelocalize(cur, image, input, reloc_armed_ > 0)) {
            ok = true;
            r.stats.inliers = static_cast<int>(cur.tracks.size());
            if (reloc_armed_ > 0) { reloc_armed_ = 0; reloc_outcome_ = Result<Pose>::success(core::fromSE3(cur.T_wb)); }
          }
          break;
        default: break;
      }
    }
    if (ok && c_.predictor) c_.predictor->update(last_, cur);
    cur.reference_kf = last_kf_id_;
    cur.T_kb = last_kf_frame_.T_wb.inverse() * cur.T_wb;

    if (phase_ == Phase::Tracking && ok && c_.keyframe_policy) {
      IKeyframePolicy::Context ctx;
      ctx.cur = &cur;
      ctx.last_kf = &last_kf_frame_;
      ctx.frames_since_kf = frames_since_kf_;
      ctx.seconds_since_kf = static_cast<double>(input.timestamp - last_kf_frame_.timestamp) * 1e-9;
      ctx.median_parallax_px = medianRotationCompensatedParallax(last_kf_frame_, cur, rig_);
      ctx.max_tracks = max_tracks_;
      ctx.optimizer_busy = optimizer_busy_;
      cur.occupied_cells = countOccupiedCells(cur);
      if (c_.keyframe_policy->needKeyframe(ctx)) {
        KeyframeRequest req;
        createKeyframe(cur, image, input, req);
        r.keyframe_id = req.kf_id.value;
        kf_request = std::move(req);
      }
    }
    if (phase_ == Phase::Tracking && ok) r.pose = PoseState{core::fromSE3(cur.T_wb), core::fromVec3(cur.v_w), {}, {}};
  }

  ++frames_since_kf_;
  r.state = phase_ == Phase::Tracking ? sm_.state() : TrackingState::NotInitialized;
  r.stats.tracks = static_cast<int>(cur.tracks.size());
  r.stats.tracks3d = cur.num3d();
  r.stats.keyframe_inserted = kf_request.has_value();
  r.debug_tracks.reserve(cur.tracks.size());
  for (const auto& t : cur.tracks) r.debug_tracks.push_back({static_cast<float>(t.uv_raw.x()), static_cast<float>(t.uv_raw.y()), t.is3d, t.id.value});
  last_ = std::move(cur);
  last_image_ = std::move(image);
  has_last_ = true;
  return r;
}

}  // namespace mvslam::frontend
