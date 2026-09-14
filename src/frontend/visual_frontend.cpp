#include "frontend/visual_frontend.hpp"

#include <algorithm>
#include <unordered_set>
#include <variant>

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
  inertial_ = inertial && rig_.hasImu();
  if (inertial_) {
    preint_kf_ = std::make_unique<sensors::imu::Preintegrator>(rig_.imu());
    preint_frame_ = std::make_unique<sensors::imu::Preintegrator>(rig_.imu());
    imu_predictor_ = dynamic_cast<ImuPredictor*>(c_.predictor.get());
  }
  // Backend workers move the map under the frontend (IMU initialization rescales/rotates it, loop closing
  // corrects keyframes, merges transform whole maps): re-anchor on the next frame.
  map_events_ = maps_.events().subscribe([this](const MapEvent& e) {
    std::lock_guard<std::mutex> lk(transform_mu_);
    auto request = [&](double scale, const char* reason) {
      if (pending_transform_) pending_transform_->scale *= scale; else pending_transform_ = PendingTransform{scale, reason};
    };
    if (const auto* ie = std::get_if<ImuInitializedEvent>(&e)) request(ie->scale, "IMU initialization");
    else if (const auto* pc = std::get_if<PosesCorrectedEvent>(&e)) request(pc->scale, "loop correction");
    else if (std::get_if<MapMergedEvent>(&e)) request(1.0, "map merge");
  });
}

VisualFrontend::~VisualFrontend() = default;

// IMU samples between the previous and this frame: piecewise-linear measurements, integrated on
// [max(t_k, t_prev), min(t_{k+1}, t_frame)] with the midpoint value (ORB-SLAM3 PreintegrateIMU boundary
// interpolation); the tail after the last sample holds it constant. Both accumulators receive the same
// intervals; measurements are rotated into the body frame when the IMU is mounted with T_bi != I.
void VisualFrontend::integrateImu(const std::vector<ImuSample>& imu, Timestamp t_frame) {
  if (!inertial_) return;
  if (last_frame_t_ == 0) {  // first frame: nothing to integrate yet
    if (!imu.empty()) last_imu_ = imu.back();
    last_frame_t_ = t_frame;
    return;
  }
  const core::Mat3 R_bi = rig_.imu().T_bi.rotationMatrix();
  std::vector<ImuSample> seq;
  if (last_imu_) seq.push_back(*last_imu_);
  seq.insert(seq.end(), imu.begin(), imu.end());
  const double t_prev = static_cast<double>(last_frame_t_) * 1e-9, t_cur = static_cast<double>(t_frame) * 1e-9;
  auto lerp = [&](const ImuSample& a, const ImuSample& b, double t, core::Vec3& acc, core::Vec3& gyro) {
    const double ta = static_cast<double>(a.timestamp) * 1e-9, tb = static_cast<double>(b.timestamp) * 1e-9;
    const double w = tb > ta ? std::clamp((t - ta) / (tb - ta), 0.0, 1.0) : 0.0;
    acc = R_bi * ((1 - w) * core::toVec3(a.acc) + w * core::toVec3(b.acc));
    gyro = R_bi * ((1 - w) * core::toVec3(a.gyro) + w * core::toVec3(b.gyro));
  };
  auto integrate = [&](const core::Vec3& acc, const core::Vec3& gyro, double dt) {
    preint_kf_->integrate(acc, gyro, dt);
    preint_frame_->integrate(acc, gyro, dt);
  };
  double covered_to = t_prev;
  for (std::size_t k = 0; k + 1 < seq.size(); ++k) {
    const double ta = static_cast<double>(seq[k].timestamp) * 1e-9, tb = static_cast<double>(seq[k + 1].timestamp) * 1e-9;
    const double lo = std::max(ta, t_prev), hi = std::min(tb, t_cur);
    if (hi <= lo) continue;
    core::Vec3 a0, w0, a1, w1;
    lerp(seq[k], seq[k + 1], lo, a0, w0);
    lerp(seq[k], seq[k + 1], hi, a1, w1);
    integrate(0.5 * (a0 + a1), 0.5 * (w0 + w1), hi - lo);
    covered_to = hi;
  }
  if (!seq.empty() && t_cur > covered_to + 1e-9) {  // hold the last sample up to the frame time
    const auto& s = seq.back();
    integrate(R_bi * core::toVec3(s.acc), R_bi * core::toVec3(s.gyro), t_cur - covered_to);
  }
  if (!imu.empty()) last_imu_ = imu.back();
  last_frame_t_ = t_frame;
}

void VisualFrontend::resetImuFromKeyframe(const Frame& kf_frame) {
  if (!inertial_) return;
  preint_kf_->reset(kf_frame.bias);
  preint_frame_->reset(kf_frame.bias);
}

// Keyframes, points and velocities were transformed on a backend worker (IMU initialization, loop
// correction, map merge). Re-anchor the frontend
// state on the (moved) reference keyframe and force a robust re-seed on this frame.
void VisualFrontend::applyMapTransform(Frame& cur) {
  PendingTransform pt;
  {
    std::lock_guard<std::mutex> lk(transform_mu_);
    if (!pending_transform_) return;
    pt = *pending_transform_;
    pending_transform_.reset();
  }
  const double scale = pt.scale;
  bool found = false;
  maps_.active()->withReadLock([&](const map::Map& m) {
    auto kf = m.keyframe(last_kf_id_);
    if (!kf) return 0;
    found = true;
    last_kf_frame_.T_wb = kf->T_wb();
    last_kf_frame_.v_w = kf->velocity();
    last_kf_frame_.bias = kf->bias();
    imu_initialized_ = m.imuState().initialized;
    return 0;
  });
  if (!found) return;
  const core::SE3 T_kb(last_.T_kb.so3(), scale * last_.T_kb.translation());
  last_.T_wb = last_kf_frame_.T_wb * T_kb;
  last_.v_w = last_kf_frame_.v_w;
  last_.bias = last_kf_frame_.bias;
  cur.v_w = last_.v_w;
  cur.bias = last_.bias;
  p3p_required_ = true;
  transformed_since_last_ = true;
  last_.T_kb = T_kb;
  if (c_.predictor) c_.predictor->reset();
  if (inertial_ && preint_kf_ && preint_frame_) {
    preint_kf_->reintegrate(last_kf_frame_.bias);   // same measurements since the keyframe, new linearization bias
    preint_frame_->reintegrate(last_.bias);
  }
  MVSLAM_INFO("frontend: map transformed ({}, scale {:.3f}), re-anchored on keyframe {}", pt.reason, scale, last_kf_id_.value);
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
    if (inertial_) {  // the inertial anchor must be the keyframe's *current* state (LBA moves it)
      if (auto kf = m.keyframe(last_kf_id_)) { last_kf_frame_.T_wb = kf->T_wb(); last_kf_frame_.v_w = kf->velocity(); last_kf_frame_.bias = kf->bias(); }
    }
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
  if (inertial_) {
    out.preint_from_last_kf = anchor_valid_from_kf_ ? preint_kf_->snapshot() : nullptr;
    resetImuFromKeyframe(cur);
    anchor_valid_from_kf_ = true;
  }
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
  lost_frames_ = 0;
  if (c_.predictor) c_.predictor->reset();
  maps_.active()->withReadLock([&](const map::Map& m) {
    if (auto kf = m.keyframe(rr.keyframe)) { last_kf_frame_.v_w = kf->velocity(); last_kf_frame_.bias = kf->bias(); }
    imu_initialized_ = m.imuState().initialized;
    return 0;
  });
  cur.v_w = last_kf_frame_.v_w;
  cur.bias = last_kf_frame_.bias;
  resetImuFromKeyframe(last_kf_frame_);
  anchor_valid_from_kf_ = false;  // the keyframe's integral restarts here, not at the keyframe time
  phase_ = Phase::Tracking;
  sm_.onRelocalized(t);
}

void VisualFrontend::restartBootstrap() {
  phase_ = Phase::Bootstrap;
  waiting_frames_ = 0;
  p3p_required_ = false;
  imu_initialized_ = false;
  if (inertial_) { preint_kf_->reset(core::Vec6::Zero()); preint_frame_->reset(core::Vec6::Zero()); }
  if (c_.initializer) c_.initializer->reset();
  if (c_.predictor) c_.predictor->reset();
  sm_.onNewMap();
}

TrackResult VisualFrontend::process(const FrameInput& input, const std::vector<ImuSample>& imu, std::optional<KeyframeRequest>& kf_request) {
  core::ScopedTimer timer("frontend.process");
  TrackResult r;
  r.timestamp = input.timestamp;
  kf_request.reset();
  if (input.images.empty() || !c_.preprocessor) { r.state = sm_.state(); return r; }

  std::shared_ptr<ImageState> image = c_.preprocessor->process(input.images[0]);
  Frame cur;
  cur.id = ++frame_counter_;
  cur.timestamp = input.timestamp;
  integrateImu(imu, input.timestamp);  // both accumulators; no-op without IMU
  if (has_last_) { cur.v_w = last_.v_w; cur.bias = last_.bias; }
  if (phase_ != Phase::Bootstrap) applyMapTransform(cur);

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
      if (inertial_) {
        cur.bias.setZero();
        cur.v_w.setZero();
        if (req.initial_map) req.initial_map->first_frame.bias.setZero();
        resetImuFromKeyframe(cur);   // the integral for the first keyframe starts here
        req.preint_from_last_kf = nullptr;
      }
      last_kf_frame_ = cur;
      last_kf_id_ = req.kf_id;
      frames_since_kf_ = 0;
      waiting_frames_ = 0;
      r.keyframe_id = req.kf_id.value;
      kf_request = std::move(req);
      phase_ = Phase::WaitingFor3d;
    }
  } else {
    const auto epoch_before = last_epoch_;
    refreshTracksFromMap(last_);
    if (inertial_) {
      imu_initialized_ = maps_.active()->withReadLock([](const map::Map& m) { return m.imuState().initialized; });
      if (imu_predictor_) imu_predictor_->setPreintegration(&preint_frame_->state(), imu_initialized_, rig_.imu().gravity);
    }
    core::SE3 predicted = c_.predictor ? c_.predictor->predict(last_, input.timestamp) : last_.T_wb;
    if (inertial_ && imu_initialized_ && imu_predictor_) {
      const auto pr = imu_predictor_->predictState(last_, input.timestamp);
      predicted = pr.T_wb;
      cur.v_w = pr.v_w;
    }
    cur.T_wb = predicted;
    if (inertial_ && c_.pose_optimizer) {  // last keyframe + IMU integral since it: the frame's inertial constraint
      anchor_.valid = imu_initialized_ && anchor_valid_from_kf_ && preint_kf_->state().dt > 0;
      anchor_.T_wb = last_kf_frame_.T_wb;
      anchor_.v_w = last_kf_frame_.v_w;
      anchor_.bias = last_kf_frame_.bias;
      anchor_.preint = anchor_.valid ? preint_kf_->snapshot() : nullptr;
      anchor_.last_ok = last_ok_;
      anchor_.map_updated = last_epoch_ != epoch_before || transformed_since_last_;
      anchor_.T_wb_last = last_.T_wb;
      anchor_.v_last = last_.v_w;
      anchor_.bias_last = last_.bias;
      anchor_.preint_last_frame = anchor_.valid ? preint_frame_->snapshot() : nullptr;
      transformed_since_last_ = false;
      c_.pose_optimizer->setInertialAnchor(&anchor_);
    }
    IFeatureTracker::Stats st;
    if (c_.tracker && last_image_) st = c_.tracker->track(*last_image_, *image, last_, cur, predicted);
    const int before_filter = static_cast<int>(cur.tracks.size());
    int filtered = 0;
    if (c_.outlier_filter && phase_ == Phase::Tracking) filtered = c_.outlier_filter->filter(last_kf_frame_, cur);
    const core::SE3 predicted_pose = cur.T_wb;
    bool ok = estimatePose(cur, st, r);
    lost_frames_ = ok ? 0 : lost_frames_ + 1;
    last_ok_ = ok;
    if (has_last_ && input.timestamp - last_.timestamp > static_cast<Timestamp>(1e9)) {  // camera outage: report the prediction quality
      const double gap = static_cast<double>(input.timestamp - last_.timestamp) * 1e-9;
      MVSLAM_INFO("frontend: {:.2f} s without images; {} prediction off by {:.3f} m from the {} estimate ({} 3-D tracks survived)", gap,
                  inertial_ && imu_initialized_ ? "IMU" : "constant-velocity", (predicted_pose.translation() - cur.T_wb.translation()).norm(),
                  ok ? "visual" : "failed visual", cur.num3d());
    }
    // Dead reckoning: with an initialized IMU the predicted state stands in for a failed visual estimate
    // (ORB-SLAM3 RECENTLY_LOST + PredictStateIMU); the state machine limits it to recently_lost_inertial_s.
    const bool dead_reckoning = !ok && inertial_ && imu_initialized_ && phase_ == Phase::Tracking;
    if (dead_reckoning) cur.T_wb = predicted;
    if (ok && inertial_) preint_frame_->reset(cur.bias);
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
      const auto state_before = sm_.state();
      const auto action = sm_.onFrame(input.timestamp, ok, kfs);
      if (sm_.state() != state_before) MVSLAM_INFO("frontend: state {} -> {} (frame {}{})", static_cast<int>(state_before), static_cast<int>(sm_.state()), cur.id, dead_reckoning ? ", IMU dead reckoning" : "");
      // Inertial dead reckoning still needs the visual map back: try relocalizing in the active map after the grace period.
      if (dead_reckoning && action == TrackingStateMachine::Action::None && lost_frames_ >= cfg_.reloc_min_lost_frames && c_.relocalizer) {
        if (tryRelocalize(cur, image, input, false)) { ok = true; r.stats.inliers = static_cast<int>(cur.tracks.size()); }
        else if (inertial_) preint_frame_->reset(cur.bias);
      }
      // Last chance before giving the map up: a descriptor-based relocalization in the active map (the first
      // frame after a camera outage often lands here because the predicted pose is off by metres).
      const bool giving_up = action == TrackingStateMachine::Action::StartNewMap || action == TrackingStateMachine::Action::ResetMap;
      if (giving_up && c_.relocalizer && tryRelocalize(cur, image, input, false)) {
        ok = true;
        r.stats.inliers = static_cast<int>(cur.tracks.size());
        MVSLAM_INFO("frontend: relocalized instead of starting a new map (frame {})", cur.id);
      }
      switch (ok ? TrackingStateMachine::Action::None : action) {
        case TrackingStateMachine::Action::StartNewMap: MVSLAM_WARN("frontend: tracking lost, starting a new map (frame {})", cur.id); maps_.startNewMap(); restartBootstrap(); break;
        case TrackingStateMachine::Action::ResetMap: MVSLAM_WARN("frontend: tracking lost, resetting the map (frame {})", cur.id); maps_.resetActive(); restartBootstrap(); break;
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
    if (dead_reckoning && !ok && sm_.state() == TrackingState::RecentlyLost && inertial_) preint_frame_->reset(cur.bias);
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
      ctx.imu_initialized = imu_initialized_;
      cur.occupied_cells = countOccupiedCells(cur);
      if (c_.keyframe_policy->needKeyframe(ctx)) {
        KeyframeRequest req;
        createKeyframe(cur, image, input, req);
        r.keyframe_id = req.kf_id.value;
        kf_request = std::move(req);
      }
    }
    if (phase_ == Phase::Tracking && (ok || (dead_reckoning && sm_.state() == TrackingState::RecentlyLost)))
      r.pose = PoseState{core::fromSE3(cur.T_wb), core::fromVec3(cur.v_w), core::fromVec3(cur.bias.head<3>()), core::fromVec3(cur.bias.tail<3>())};
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
