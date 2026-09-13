#include "backend/mapper/mapper.hpp"

#include <algorithm>
#include <cmath>

#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::backend {

Mapper::Mapper(const Config& cfg, const sensors::CameraRig& rig, map::MapStore& maps) : cfg_(cfg), rig_(rig), maps_(maps), matcher_(cfg, rig) {}

void Mapper::setDescriptorFrom(map::MapPoint& mp, const map::KeyFrame& kf, int index, int stride) {
  const auto& d = kf.descriptors();
  const std::size_t off = static_cast<std::size_t>(index) * static_cast<std::size_t>(stride);
  if (stride <= 0 || d.size() < off + static_cast<std::size_t>(stride)) return;
  mp.setDescriptor(std::vector<std::uint8_t>(d.begin() + static_cast<long>(off), d.begin() + static_cast<long>(off + static_cast<std::size_t>(stride))));
}

bool Mapper::triangulatePending(map::Map& m, const std::vector<map::PendingObservation>& obs, core::Vec3& X_w) const {
  if (obs.size() < 2) return false;
  const auto& cam = rig_.camera(0);
  auto kf0 = m.keyframe(obs.front().kf);
  auto kf1 = m.keyframe(obs.back().kf);
  if (!kf0 || !kf1 || kf0->bad() || kf1->bad()) return false;
  const auto& kp0 = kf0->keypoints()[static_cast<std::size_t>(obs.front().keypoint_index)];
  const auto& kp1 = kf1->keypoints()[static_cast<std::size_t>(obs.back().keypoint_index)];
  const core::SE3 T_wc0 = kf0->T_wb() * rig_.T_bc(0), T_wc1 = kf1->T_wb() * rig_.T_bc(0);
  const core::Vec3 r0 = T_wc0.rotationMatrix() * cam.unprojectUndistorted(kp0.uv).normalized();
  const core::Vec3 r1 = T_wc1.rotationMatrix() * cam.unprojectUndistorted(kp1.uv).normalized();
  const double cos_parallax = r0.dot(r1);
  if (cos_parallax > std::cos(cfg_.backend.triangulation_min_parallax_deg * M_PI / 180.0)) return false;
  // Midpoint of the closest points between the two rays.
  const core::Vec3 o0 = T_wc0.translation(), o1 = T_wc1.translation();
  const core::Vec3 w = o0 - o1;
  const double a = r0.dot(r0), b = r0.dot(r1), c = r1.dot(r1), d = r0.dot(w), e = r1.dot(w);
  const double det = a * c - b * b;
  if (std::abs(det) < 1e-12) return false;
  const double s = (b * e - c * d) / det, t = (a * e - b * d) / det;
  if (s <= 0 || t <= 0) return false;
  X_w = 0.5 * ((o0 + s * r0) + (o1 + t * r1));
  // Reprojection check in both keyframes.
  for (const auto* kf : {kf0.get(), kf1.get()}) {
    const core::Vec3 pc = rig_.T_cb(0) * (kf->T_wb().inverse() * X_w);
    if (pc.z() <= cfg_.frontend.min_depth_m) return false;
    const auto& kp = kf == kf0.get() ? kp0 : kp1;
    if ((cam.project(pc) - kp.uv).norm() > cfg_.backend.triangulation_max_reproj_px) return false;
  }
  return true;
}

void Mapper::cullRecentMapPoints(map::Map& m, core::KeyframeId latest) {
  const int grace = cfg_.backend.mp_cull_grace_keyframes;
  while (!recent_points_.empty()) {
    const auto [created_at, mpid] = recent_points_.front();
    if (latest.value < created_at.value + static_cast<std::uint64_t>(grace)) break;
    recent_points_.pop_front();
    auto mp = m.mapPoint(mpid);
    if (mp && !mp->bad() && mp->numObservations() < cfg_.backend.mp_cull_min_observations) {
      m.removeMapPoint(mpid);
      ++stats_.points_culled;
    }
  }
}

void Mapper::insertInitialMap(map::Map& m, const frontend::KeyframeRequest& req) {
  const auto& init = *req.initial_map;
  auto kf1 = std::make_shared<map::KeyFrame>(init.first_kf, init.first_frame.timestamp, init.first_frame.T_wb);
  std::unordered_map<core::TrackId, int> idx1;
  for (const auto& t : init.first_frame.tracks) {
    map::Keypoint kp;
    kp.uv_raw = t.uv_raw; kp.uv = t.uv; kp.track = t.id; kp.info = t.info;
    idx1[t.id] = static_cast<int>(kf1->keypoints().size());
    kf1->keypoints().push_back(kp);
  }
  if (descriptors_) {
    std::vector<core::Vec2> uv;
    for (const auto& t : init.first_frame.tracks) uv.push_back(t.uv_raw);
    kf1->descriptors() = descriptors_->describe(init.first_left_raw.view(), uv);
  }
  kf1->mapPointSlots().assign(kf1->keypoints().size(), core::MapPointId{});
  m.addKeyFrame(kf1);
  m.setLastKeyframe(kf1->id());
  // The second keyframe is inserted by the regular path; the points exist first so it observes them.
  for (const auto& [tid, X_w] : init.points) {
    const auto mpid = frontend::toMapPointId(tid);
    auto mp = m.addMapPoint(std::make_shared<map::MapPoint>(mpid, X_w, kf1->id()));
    if (auto it = idx1.find(tid); it != idx1.end()) { m.addObservation(mpid, kf1->id(), {it->second, 0}); if (descriptors_) setDescriptorFrom(*mp, *kf1, it->second, descriptors_->stride()); }
    recent_points_.emplace_back(kf1->id(), mpid);
  }
  stats_.points_triangulated += static_cast<int>(init.points.size());
}

core::KeyframeId Mapper::process(frontend::KeyframeRequest req, std::stop_token st) {
  core::ScopedTimer timer("mapper.process");
  (void)st;
  auto map = maps_.active();
  const bool empty = map->withReadLock([](const map::Map& m) { return m.numKeyframes() == 0; });
  if (!req.initial && empty) return core::KeyframeId{};  // map was reset after this request was made

  const auto& tracks = req.frame.tracks;
  std::vector<frontend::DepthMeasurement> depth(tracks.size());
  if (depth_) depth = depth_->measure(req);
  std::vector<std::uint8_t> desc;
  if (descriptors_) {
    std::vector<core::Vec2> uv_raw;
    uv_raw.reserve(tracks.size());
    for (const auto& t : tracks) uv_raw.push_back(t.uv_raw);
    desc = descriptors_->describe(req.left_raw.view(), uv_raw);
  }
  auto kf = std::make_shared<map::KeyFrame>(req.kf_id, req.frame.timestamp, req.frame.T_wb);
  auto& kps = kf->keypoints();
  kps.reserve(tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    map::Keypoint kp;
    kp.uv_raw = tracks[i].uv_raw; kp.uv = tracks[i].uv; kp.cam = 0; kp.track = tracks[i].id; kp.info = tracks[i].info;
    kp.depth = depth[i].depth; kp.has_right = depth[i].has_right; kp.uv_right = depth[i].uv_right;
    kps.push_back(kp);
  }
  kf->descriptors() = std::move(desc);
  kf->mapPointSlots().assign(tracks.size(), core::MapPointId{});
  const auto& cam = rig_.camera(0);
  const core::SE3 T_wc0 = req.frame.T_wb * rig_.T_bc(0);

  int from_depth = 0, triangulated = 0;
  const int stride = descriptors_ ? descriptors_->stride() : 0;
  LocalMapMatcher::Stats lm;
  map->withWriteLock([&](map::Map& m) {
    if (req.initial_map) insertInitialMap(m, req);
    m.addKeyFrame(kf);
    if (auto prev = m.keyframe(m.lastKeyframe())) { prev->setNextKf(kf->id()); kf->setPrevKf(prev->id()); }
    m.setLastKeyframe(kf->id());
    for (std::size_t i = 0; i < kps.size(); ++i) {
      const auto mpid = m.resolveTrack(kps[i].track);  // follows local-map aliases (identity otherwise)
      kps[i].track = core::TrackId{mpid.value};
      auto mp = m.mapPoint(mpid);
      if (mp && !mp->bad()) {  // re-observation; LBA arbitrates outliers (a gate here would leave stale points alive)
        m.addObservation(mpid, kf->id(), {static_cast<int>(i), 0});
        setDescriptorFrom(*mp, *kf, static_cast<int>(i), stride);
        continue;
      }
      if (kps[i].depth > 0) {
        const core::Vec3 X_w = T_wc0 * (kps[i].depth * cam.unprojectUndistorted(kps[i].uv));
        auto created = m.addMapPoint(std::make_shared<map::MapPoint>(mpid, X_w, kf->id()));
        m.addObservation(mpid, kf->id(), {static_cast<int>(i), 0});
        setDescriptorFrom(*created, *kf, static_cast<int>(i), stride);
        m.pending2d().erase(kps[i].track);
        recent_points_.emplace_back(kf->id(), mpid);
        ++from_depth;
        continue;
      }
      auto& pending = m.pending2d()[kps[i].track];
      pending.erase(std::remove_if(pending.begin(), pending.end(), [&](const map::PendingObservation& o) { return !m.keyframe(o.kf); }), pending.end());
      pending.push_back({kf->id(), static_cast<int>(i)});
      core::Vec3 X_w;
      if (triangulatePending(m, pending, X_w)) {
        auto created = m.addMapPoint(std::make_shared<map::MapPoint>(mpid, X_w, pending.front().kf));
        for (const auto& o : pending) m.addObservation(mpid, o.kf, {o.keypoint_index, 0});
        setDescriptorFrom(*created, *kf, static_cast<int>(i), stride);
        m.pending2d().erase(kps[i].track);
        recent_points_.emplace_back(kf->id(), mpid);
        ++triangulated;
      }
    }
    // Forget 2-D observations whose track died long ago (never triangulated).
    for (auto it = m.pending2d().begin(); it != m.pending2d().end();) {
      if (kf->id().value > it->second.back().kf.value + 10) it = m.pending2d().erase(it); else ++it;
    }
    m.updateCovisibility(kf->id(), cfg_.backend.lba_min_covisibility);
    if (stride > 0 && cfg_.backend.local_match_radius_px > 0) {
      lm = matcher_.match(m, *kf, stride);
      if (lm.matched > 0) m.updateCovisibility(kf->id(), cfg_.backend.lba_min_covisibility);
    }
    cullRecentMapPoints(m, kf->id());
    return 0;
  });
  stats_.local_matches += lm.matched;
  stats_.migrated_observations += lm.migrated;
  stats_.keyframes++;
  stats_.points_from_depth += from_depth;
  stats_.points_triangulated += triangulated;
  MVSLAM_DEBUG("mapper: kf {} tracks={} depth-points={} triangulated={} local-matches={}/{} migrated={}", kf->id().value, tracks.size(), from_depth,
               triangulated, lm.matched, lm.candidates, lm.migrated);
  if (maps_.events().hasSubscribers())
    maps_.events().publish(KeyframeInsertedEvent{map->id().value, kf->id().value, req.frame.timestamp, core::fromSE3(req.frame.T_wb)});
  return kf->id();
}

}  // namespace mvslam::backend
