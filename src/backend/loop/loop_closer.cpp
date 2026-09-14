#include "backend/loop/loop_closer.hpp"

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <unordered_set>

#include "backend/mapper/local_map_matcher.hpp"  // hammingDistance
#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::backend::loop {

namespace {
constexpr int kStride = 32;

struct LocalPoint { core::MapPointId id; core::Vec3 X_w; std::vector<std::uint8_t> desc; };
struct Neighbourhood { bool ok = false; core::SE3 T_wb; std::vector<core::KeyframeId> keyframes; std::vector<LocalPoint> points; };
struct KeyframeView {
  core::KeyframeId id;
  core::SE3 T_wb;
  std::vector<core::Vec2> uv, uv_raw;
  std::vector<bool> usable;
  std::vector<std::uint8_t> desc;
  std::vector<core::MapPointId> slots;
};

bool allZero(const std::uint8_t* d) { for (int i = 0; i < kStride; ++i) if (d[i]) return false; return true; }

core::SE3 transformPose(const core::Sim3& S, const core::SE3& T) { return core::SE3(S.rxso3().so3() * T.so3(), S * T.translation()); }

// Best keypoint among `cands` (all when null) for descriptor `d`: Hamming <= max, ratio 0.8 against the second best.
int bestMatch(const std::uint8_t* d, const std::vector<std::uint8_t>& desc, const std::vector<int>* cands, const std::vector<bool>& used,
              int max_hamming, int& best_dist) {
  int best = std::numeric_limits<int>::max(), second = best, best_i = -1;
  auto consider = [&](int i) {
    if (used[static_cast<std::size_t>(i)]) return;
    const std::uint8_t* q = desc.data() + static_cast<std::size_t>(i) * kStride;
    if (allZero(q)) return;
    const int h = hammingDistance(d, q, kStride);
    if (h < best) { second = best; best = h; best_i = i; } else if (h < second) { second = h; }
  };
  if (cands) for (int i : *cands) consider(i);
  else for (int i = 0; i < static_cast<int>(desc.size() / kStride); ++i) consider(i);
  if (best_i < 0 || best > max_hamming || (second < std::numeric_limits<int>::max() && best > 0.8 * second)) return -1;
  best_dist = best;
  return best_i;
}

KeyframeView view(const map::KeyFrame& kf) {
  KeyframeView v;
  v.id = kf.id();
  v.T_wb = kf.T_wb();
  v.desc = kf.descriptors();
  v.slots = kf.mapPointSlots();
  const auto& kps = kf.keypoints();
  v.uv.reserve(kps.size());
  v.uv_raw.reserve(kps.size());
  v.usable.reserve(kps.size());
  for (const auto& kp : kps) { v.uv.push_back(kp.uv); v.uv_raw.push_back(kp.uv_raw); v.usable.push_back(kp.cam == 0 && !kp.outlier); }
  if (v.slots.size() < kps.size()) v.slots.resize(kps.size());
  if (v.desc.size() < kps.size() * kStride) v.desc.assign(kps.size() * kStride, 0);  // no descriptors: nothing matches
  return v;
}

// Candidate keyframe + its strongest covisibles and their (described) map points.
Neighbourhood collect(const map::Map& m, core::KeyframeId center, int max_covisibles) {
  Neighbourhood nb;
  auto kf = m.keyframe(center);
  if (!kf || kf->bad()) return nb;
  nb.ok = true;
  nb.T_wb = kf->T_wb();
  nb.keyframes.push_back(center);
  int n = 0;
  for (const auto& [k, w] : m.covisibility().covisibles(center, 1)) { if (n++ >= max_covisibles) break; nb.keyframes.push_back(k); }
  std::unordered_set<core::MapPointId> seen;
  for (auto k : nb.keyframes) {
    auto lk = m.keyframe(k);
    if (!lk || lk->bad()) continue;
    for (auto id : lk->mapPointSlots()) {
      if (!id.valid() || !seen.insert(id).second) continue;
      auto mp = m.mapPoint(id);
      if (!mp || mp->bad() || mp->descriptor().size() < static_cast<std::size_t>(kStride)) continue;
      nb.points.push_back({id, mp->position(), mp->descriptor()});
    }
  }
  return nb;
}

// Points (mapped by `S` into the view's world frame) projected into a keyframe view and matched by descriptor
// within `radius` (unique per keypoint). Returns the match count; `out` receives (keypoint, point index).
int projectedMatches(const KeyframeView& v, const std::vector<LocalPoint>& points, const core::Sim3& S, const sensors::ICameraModel& cam,
                     const core::SE3& T_cb, double radius, int max_hamming, double min_depth, std::vector<std::pair<int, std::size_t>>* out) {
  const core::SE3 T_cw = T_cb * v.T_wb.inverse();
  std::vector<bool> used(v.uv.size(), false);
  for (std::size_t i = 0; i < used.size(); ++i) used[i] = !v.usable[i];
  const double r2 = radius * radius;
  std::vector<int> near;
  int n = 0;
  for (std::size_t p = 0; p < points.size(); ++p) {
    const core::Vec3 pc = T_cw * (S * points[p].X_w);
    if (pc.z() <= min_depth) continue;
    const core::Vec2 proj = cam.project(pc);
    if (!cam.inImage(proj, 1)) continue;
    near.clear();
    for (std::size_t i = 0; i < v.uv.size(); ++i) if (!used[i] && (v.uv[i] - proj).squaredNorm() <= r2) near.push_back(static_cast<int>(i));
    if (near.empty()) continue;
    int dist = 0;
    const int best = bestMatch(points[p].desc.data(), v.desc, &near, used, max_hamming, dist);
    if (best < 0) continue;
    used[static_cast<std::size_t>(best)] = true;
    ++n;
    if (out) out->emplace_back(best, p);
  }
  return n;
}

struct FuseStats { int replaced = 0, added = 0; };

// ORB-SLAM3 SearchAndFuse: loop points projected into the (already corrected) window keyframes; a keypoint that
// holds a younger map point is re-pointed to the loop point (merge + alias), an empty keypoint gains an
// observation (+ alias for its live track so the frontend and mapper follow).
FuseStats fuseInto(map::Map& m, const std::vector<core::KeyframeId>& window, const std::vector<LocalPoint>& loop_points, const sensors::ICameraModel& cam,
                   const core::SE3& T_cb, double radius, int max_hamming, double min_depth) {
  FuseStats st;
  std::unordered_set<core::MapPointId> loop_ids;
  for (const auto& lp : loop_points) loop_ids.insert(lp.id);
  for (auto k : window) {
    auto kf = m.keyframe(k);
    if (!kf || kf->bad()) continue;
    const KeyframeView v = view(*kf);
    std::vector<std::pair<int, std::size_t>> matches;
    projectedMatches(v, loop_points, core::Sim3(), cam, T_cb, radius, max_hamming, min_depth, &matches);
    for (const auto& [kp, p] : matches) {
      const auto& lp = loop_points[p];
      auto loop_mp = m.mapPoint(lp.id);
      if (!loop_mp || loop_mp->bad()) continue;
      const auto slot = kf->mapPointSlots()[static_cast<std::size_t>(kp)];
      if (slot == lp.id) continue;
      if (slot.valid()) {
        if (loop_ids.count(slot)) continue;        // two loop points on one keypoint: ambiguous, leave it
        m.mergeMapPoints(lp.id, slot);
        ++st.replaced;
      } else {
        if (loop_mp->observations().count(k)) continue;   // the loop point already sits on another keypoint of this keyframe
        m.addObservation(lp.id, k, {kp, 0});
        const auto track = kf->keypoints()[static_cast<std::size_t>(kp)].track;
        kf->keypoints()[static_cast<std::size_t>(kp)].track = core::TrackId{lp.id.value};
        m.addTrackAlias(track, lp.id);
        ++st.added;
      }
    }
  }
  return st;
}
}  // namespace

// ---------------------------------------------------------------------------------------------------
void LoopDetector::closed(core::KeyframeId query) {
  cooldown_until_ = query.value + static_cast<std::uint64_t>(std::max(0, cfg_.loop.cooldown_keyframes));
  memory_ = {};
}

std::optional<LoopHypothesis> LoopDetector::detect(core::KeyframeId query) {
  core::ScopedTimer timer("loop.detect");
  ++stats_.queries;
  if (query.value < cooldown_until_) return std::nullopt;
  auto* recognizer = maps_.placeRecognizer();
  if (!recognizer) return std::nullopt;
  auto active = maps_.active();
  const auto& cam = rig_.camera(0);
  const core::SE3 T_cb = rig_.T_cb(0);
  const auto& L = cfg_.loop;
  const double min_depth = cfg_.frontend.min_depth_m;

  // ---- query snapshot (read lock) -----------------------------------------------------------------
  KeyframeView q;
  std::vector<KeyframeView> q_cov;
  std::vector<core::KeyframeId> exclude;
  std::unordered_map<core::MapPointId, core::Vec3> q_points;
  bool inertial = false;
  const bool have = active->withReadLock([&](const map::Map& m) {
    auto kf = m.keyframe(query);
    if (!kf || kf->bad() || kf->descriptors().size() < static_cast<std::size_t>(kStride)) return false;
    q = view(*kf);
    exclude.push_back(query);
    for (const auto& [k, w] : m.covisibility().covisibles(query, 1)) {
      exclude.push_back(k);
      if (static_cast<int>(q_cov.size()) < L.vote_keyframes) if (auto o = m.keyframe(k); o && !o->bad()) q_cov.push_back(view(*o));
    }
    const auto kfs = m.keyframes();  // ascending ids
    for (auto it = kfs.rbegin(); it != kfs.rend(); ++it) {
      if ((*it)->id().value + static_cast<std::uint64_t>(std::max(0, L.min_keyframe_gap)) <= query.value) break;
      if ((*it)->id() != query) exclude.push_back((*it)->id());
    }
    for (auto s : kf->mapPointSlots()) if (s.valid()) if (auto mp = m.mapPoint(s); mp && !mp->bad()) q_points[s] = mp->position();
    inertial = m.imuState().initialized;
    return true;
  });
  if (!have) return std::nullopt;
  const auto candidates = recognizer->query(q.desc, kStride, exclude, L.max_candidates);
  stats_.candidates += static_cast<int>(candidates.size());
  bool pending = false;
  for (const auto& cand : candidates) {
    const auto cand_map_id = maps_.resolveMap(cand.map);
    auto cmap = maps_.find(cand_map_id);
    if (!cmap) continue;
    Neighbourhood nb;
    bool cand_inertial = false;
    cmap->withReadLock([&](const map::Map& m) { nb = collect(m, cand.keyframe, 10); cand_inertial = m.imuState().initialized; return 0; });
    if (!nb.ok || static_cast<int>(nb.points.size()) < L.min_matches) continue;

    // ---- 1. descriptor matches: candidate local points -> query keypoints, unique per keypoint ----
    std::vector<bool> used(q.uv.size(), false);
    for (std::size_t i = 0; i < used.size(); ++i) used[i] = !q.usable[i];
    struct Match { std::size_t p; int dist; };
    std::map<int, Match> by_kp;  // ordered: deterministic
    for (std::size_t p = 0; p < nb.points.size(); ++p) {
      int dist = 0;
      const int kp = bestMatch(nb.points[p].desc.data(), q.desc, nullptr, used, L.max_hamming, dist);
      if (kp < 0) continue;
      auto it = by_kp.find(kp);
      if (it == by_kp.end() || dist < it->second.dist) by_kp[kp] = {p, dist};
    }
    MVSLAM_DEBUG("loop: kf {} candidate kf {} (map {}, score {:.2f}): local points {} matches {}", query.value, cand.keyframe.value, cand_map_id.value,
                 cand.score, nb.points.size(), by_kp.size());
    if (static_cast<int>(by_kp.size()) < L.min_matches) continue;

    // ---- 2. P3P-RANSAC + motion-only BA: query body pose in the candidate's world ----
    frontend::Frame f;
    std::vector<int> kp_of_track;
    for (const auto& [kp, mt] : by_kp) {
      frontend::Track t;
      t.id = core::TrackId{nb.points[mt.p].id.value};
      t.uv = q.uv[static_cast<std::size_t>(kp)];
      t.uv_raw = q.uv_raw[static_cast<std::size_t>(kp)];
      t.is3d = true;
      t.X_w = nb.points[mt.p].X_w;
      f.tracks.push_back(t);
      kp_of_track.push_back(kp);
      used[static_cast<std::size_t>(kp)] = true;
    }
    core::SE3 T;
    std::vector<int> outliers;
    if (!p3p_.solve(f, T, outliers)) { MVSLAM_DEBUG("loop: p3p failed ({} matches)", f.tracks.size()); continue; }
    f.T_wb = T;
    for (int i : outliers) f.tracks[static_cast<std::size_t>(i)].is3d = false;
    auto po = pose_optimizer_.optimize(f, 0);
    if (!po.ok && po.inliers < 15) { MVSLAM_DEBUG("loop: pose refinement rejected ({} inliers)", po.inliers); continue; }

    // ---- 3. projection expansion over the candidate's local map, re-optimization ----
    {
      const core::SE3 T_cw = T_cb * f.T_wb.inverse();
      std::unordered_set<core::MapPointId> have_pt;
      for (const auto& t : f.tracks) have_pt.insert(core::MapPointId{t.id.value});
      const double r2 = L.match_radius_px * L.match_radius_px;
      std::vector<int> near;
      for (const auto& lp : nb.points) {
        if (have_pt.count(lp.id)) continue;
        const core::Vec3 pc = T_cw * lp.X_w;
        if (pc.z() <= min_depth) continue;
        const core::Vec2 proj = cam.project(pc);
        if (!cam.inImage(proj, 1)) continue;
        near.clear();
        for (std::size_t i = 0; i < q.uv.size(); ++i) if (!used[i] && (q.uv[i] - proj).squaredNorm() <= r2) near.push_back(static_cast<int>(i));
        if (near.empty()) continue;
        int dist = 0;
        const int kp = bestMatch(lp.desc.data(), q.desc, &near, used, L.max_hamming, dist);
        if (kp < 0) continue;
        frontend::Track t;
        t.id = core::TrackId{lp.id.value};
        t.uv = q.uv[static_cast<std::size_t>(kp)];
        t.uv_raw = q.uv_raw[static_cast<std::size_t>(kp)];
        t.is3d = true;
        t.X_w = lp.X_w;
        f.tracks.push_back(t);
        kp_of_track.push_back(kp);
        used[static_cast<std::size_t>(kp)] = true;
      }
    }
    po = pose_optimizer_.optimize(f, 0);
    if (po.inliers < L.min_inliers) { MVSLAM_DEBUG("loop: {} inliers after expansion (< {})", po.inliers, L.min_inliers); continue; }
    ++stats_.verified;

    // ---- 4. candidate-world <- query-world (scale from 3-D/3-D pairs for monocular maps) ----
    const core::SO3 R = f.T_wb.so3() * q.T_wb.so3().inverse();
    double s = 1.0;
    if (cfg_.sensor == SensorMode::Mono && !(inertial && cand_inertial)) {
      std::vector<double> ratios;
      for (std::size_t i = 0; i < f.tracks.size(); ++i) {
        if (!f.tracks[i].is3d) continue;
        const auto slot = q.slots[static_cast<std::size_t>(kp_of_track[i])];
        auto it = slot.valid() ? q_points.find(slot) : q_points.end();
        if (it == q_points.end()) continue;
        const double dq = (it->second - q.T_wb.translation()).norm(), dc = (f.tracks[i].X_w - f.T_wb.translation()).norm();
        if (dq > 1e-6 && dc > 1e-6) ratios.push_back(dc / dq);
      }
      if (ratios.size() >= 10) {
        std::nth_element(ratios.begin(), ratios.begin() + static_cast<std::ptrdiff_t>(ratios.size() / 2), ratios.end());
        s = ratios[ratios.size() / 2];
      }
    }
    const core::Vec3 t = f.T_wb.translation() - s * (R * q.T_wb.translation());
    const core::Sim3 S_cq(Sophus::RxSO3d(s, R), t);

    // ---- 5. inertial gate: both maps gravity-aligned -> the correction must be (nearly) yaw-only ----
    if (inertial && cand_inertial) {
      const core::Vec3 phi = R.log();
      if (std::abs(phi.x()) > L.inertial_max_roll_pitch_rad || std::abs(phi.y()) > L.inertial_max_roll_pitch_rad) {
        MVSLAM_INFO("loop: kf {} ~ kf {} rejected by the inertial gate (roll {:.4f} pitch {:.4f} rad)", query.value, cand.keyframe.value, phi.x(), phi.y());
        continue;
      }
    }

    // ---- 6. spatial votes: the query's covisibles see the candidate's points under the same transform ----
    const core::Sim3 S_qc = S_cq.inverse();
    int spatial = 0;
    for (const auto& v : q_cov)
      if (projectedMatches(v, nb.points, S_qc, cam, T_cb, L.match_radius_px, L.max_hamming, min_depth, nullptr) >= L.vote_min_matches) ++spatial;

    // ---- 7. temporal carry-over: consecutive queries agreeing on the same place ----
    int carried = 0;
    if (memory_.valid && memory_.map == cand_map_id) {
      const bool same_place = memory_.candidate == cand.keyframe || std::find(nb.keyframes.begin(), nb.keyframes.end(), memory_.candidate) != nb.keyframes.end() ||
                              std::find(memory_.neighbourhood.begin(), memory_.neighbourhood.end(), cand.keyframe) != memory_.neighbourhood.end();
      if (same_place) carried = memory_.votes;
    }
    const int total = 1 + spatial + carried;
    MVSLAM_INFO("loop: kf {} ~ kf {} (map {}): matches {} inliers {} votes {} (spatial {} + carried {}) scale {:.3f}{}", query.value, cand.keyframe.value,
                cand_map_id.value, by_kp.size(), po.inliers, total, spatial, carried, s, total >= L.min_votes ? " -> accepted" : " -> pending");
    if (total >= L.min_votes) {
      LoopHypothesis h;
      h.query = query;
      h.candidate = cand.keyframe;
      h.query_map = active->id();
      h.candidate_map = cand_map_id;
      h.S_cq = S_cq;
      h.matches = static_cast<int>(by_kp.size());
      h.inliers = po.inliers;
      h.votes = total;
      for (std::size_t i = 0; i < f.tracks.size(); ++i) if (f.tracks[i].is3d) h.inlier_matches.emplace_back(kp_of_track[i], core::MapPointId{f.tracks[i].id.value});
      memory_ = {};
      ++stats_.accepted;
      return h;
    }
    memory_ = Memory{true, cand.keyframe, cand_map_id, nb.keyframes, 1 + spatial, 0};
    pending = true;
    break;
  }
  if (!pending && memory_.valid && ++memory_.misses > 1) memory_ = {};
  return std::nullopt;
}

// ---------------------------------------------------------------------------------------------------
LoopCloser::Report LoopCloser::close(const LoopHypothesis& h, std::stop_token st) {
  core::ScopedTimer timer("loop.close");
  return h.candidate_map == h.query_map ? correctLoop(h, st) : merge(h, st);
}

void LoopCloser::publishCorrections(map::Map& map, const std::unordered_map<core::KeyframeId, core::SE3>& before, double scale) {
  PosesCorrectedEvent ev;
  ev.map_id = map.id().value;
  ev.scale = scale;
  map.withReadLock([&](const map::Map& m) {
    for (const auto& [k, T_old] : before) {
      auto kf = m.keyframe(k);
      if (!kf || kf->bad()) continue;
      const core::SE3& T_new = kf->T_wb();
      if ((T_new.inverse() * T_old).log().squaredNorm() < 1e-18) continue;
      ev.corrections.push_back(PoseCorrection{k.value, core::fromSE3(T_old), core::fromSE3(T_new)});
    }
    return 0;
  });
  std::sort(ev.corrections.begin(), ev.corrections.end(), [](const PoseCorrection& a, const PoseCorrection& b) { return a.keyframe_id < b.keyframe_id; });
  maps_.events().publish(ev);
  maps_.events().publish(MapPointsUpdatedEvent{map.id().value, map.changeEpoch()});
}

LoopCloser::Report LoopCloser::correctLoop(const LoopHypothesis& h, std::stop_token st) {
  Report rep;
  rep.map = h.query_map;
  auto map = maps_.find(h.query_map);
  if (!map) return rep;
  const auto& cam = rig_.camera(0);
  const core::SE3 T_cb = rig_.T_cb(0);
  const auto& L = cfg_.loop;
  const double s = h.S_cq.scale();
  const core::SO3 R = h.S_cq.rxso3().so3();
  std::unordered_map<core::KeyframeId, core::SE3> reference;
  std::vector<std::pair<core::KeyframeId, core::KeyframeId>> loop_edges;
  std::vector<core::KeyframeId> window;
  bool ok = false, inertial = false;
  map->withWriteLock([&](map::Map& m) {
    auto q = m.keyframe(h.query);
    auto c = m.keyframe(h.candidate);
    if (!q || !c || q->bad() || c->bad()) return 0;
    inertial = m.imuState().initialized;
    for (const auto& kf : m.keyframes()) if (!kf->bad()) reference[kf->id()] = kf->T_wb();
    window.push_back(h.query);
    for (const auto& [k, w] : m.covisibility().covisibles(h.query, 1)) window.push_back(k);
    const std::set<core::KeyframeId> wset(window.begin(), window.end());
    if (wset.count(h.candidate)) return 0;   // already connected: nothing to correct
    std::unordered_map<core::KeyframeId, std::set<core::KeyframeId>> before;
    for (auto k : window) for (const auto& [o, w] : m.covisibility().neighbours(k)) before[k].insert(o);
    // Rigid (or similarity) correction of the query window and the points it observes.
    std::unordered_set<core::MapPointId> moved;
    for (auto k : window) {
      auto kf = m.keyframe(k);
      if (!kf || kf->bad()) continue;
      kf->setT_wb(transformPose(h.S_cq, kf->T_wb()));
      kf->setVelocity(s * (R * kf->velocity()));
      for (auto id : kf->mapPointSlots())
        if (id.valid() && moved.insert(id).second) if (auto mp = m.mapPoint(id); mp && !mp->bad()) mp->setPosition(h.S_cq * mp->position());
    }
    const Neighbourhood nb = collect(m, h.candidate, 30);
    const FuseStats fs = fuseInto(m, window, nb.points, cam, T_cb, L.match_radius_px, L.max_hamming, cfg_.frontend.min_depth_m);
    rep.fused = fs.replaced;
    rep.new_observations = fs.added;
    for (auto k : window) m.updateCovisibility(k, cfg_.backend.lba_min_covisibility);
    q->loopEdges().insert(h.candidate);
    c->loopEdges().insert(h.query);
    loop_edges.emplace_back(h.query, h.candidate);
    for (auto k : window)  // new strong connections out of the window: loop connections (measured with corrected poses)
      for (const auto& [o, w] : m.covisibility().neighbours(k))
        if (!wset.count(o) && w >= L.strong_covisibility && !before[k].count(o)) loop_edges.emplace_back(k, o);
    m.bumpCorrectionEpoch();
    ok = true;
    return 0;
  });
  if (!ok) return rep;
  rep.window = static_cast<int>(window.size());
  rep.loop_edges = static_cast<int>(loop_edges.size());
  // A revisit that is already consistent (a closure a few keyframes earlier fixed the drift) only needs the
  // fusion; the pose graph would not move (translation < 1 cm, rotation < 0.05 deg).
  const bool negligible = h.S_cq.translation().norm() < 0.01 && R.log().norm() < 1e-3 && std::abs(s - 1.0) < 1e-4;
  if (!negligible) {
    PgoOptions po;
    po.iterations = L.pgo_iters;
    po.strong_weight = L.strong_covisibility;
    po.inertial_chain = inertial;
    rep.pgo = pgo_.optimize(*map, reference, {h.candidate}, loop_edges, po);
  }
  if (weld_ba_ && L.weld_ba && !inertial) weld_ba_->run(*map, h.query, st, nullptr);
  publishCorrections(*map, reference, s);
  maps_.events().publish(LoopClosedEvent{map->id().value, h.query.value, h.candidate.value});
  rep.closed = true;
  if (negligible)
    MVSLAM_INFO("loop closed: kf {} -> kf {} (map {}): window {} kfs, fused {} points, {} new observations (correction negligible, pgo skipped)",
                h.query.value, h.candidate.value, map->id().value, rep.window, rep.fused, rep.new_observations);
  else
    MVSLAM_INFO("loop closed: kf {} -> kf {} (map {}): correction {:.3f} m / {:.2f} deg, window {} kfs, fused {} points, {} new observations, {} loop edges, pgo {} nodes / {} edges chi2 {:.1f} -> {:.1f}",
                h.query.value, h.candidate.value, map->id().value, h.S_cq.translation().norm(), R.log().norm() * 180.0 / M_PI, rep.window, rep.fused,
                rep.new_observations, rep.loop_edges, rep.pgo.nodes, rep.pgo.edges, rep.pgo.initial_chi2, rep.pgo.final_chi2);
  return rep;
}

LoopCloser::Report LoopCloser::merge(const LoopHypothesis& h, std::stop_token st) {
  Report rep;
  rep.map = h.candidate_map;
  auto A = maps_.find(h.query_map);
  auto B = maps_.find(h.candidate_map);
  if (!A || !B || A == B || maps_.active() != A) return rep;   // only the active map merges into a stored one
  const auto& cam = rig_.camera(0);
  const core::SE3 T_cb = rig_.T_cb(0);
  const auto& L = cfg_.loop;
  std::unordered_map<core::KeyframeId, core::SE3> old_poses;
  FuseStats fs;
  bool ok = false, b_inertial = false, moved_active = true;
  double scale = 1.0;
  int moved_kfs = 0, window_size = 0;
  A->withWriteLock([&](map::Map& a) {
    B->withWriteLock([&](map::Map& b) {
      auto q = a.keyframe(h.query);
      auto c = b.keyframe(h.candidate);
      if (!q || !c || q->bad() || c->bad()) return 0;
      core::Sim3 S = h.S_cq;   // B <- A
      map::Map* moving = &a;
      if (a.imuState().initialized && !b.imuState().initialized) { moving = &b; S = h.S_cq.inverse(); moved_active = false; }  // gravity-aligned frame wins
      for (const auto& kf : moving->keyframes()) if (!kf->bad()) old_poses[kf->id()] = kf->T_wb();
      moving->applyTransform(S);
      scale = S.scale();
      moved_kfs = static_cast<int>(old_poses.size());
      const auto last = a.lastKeyframe();
      b.absorb(a);
      b.setLastKeyframe(last);
      std::vector<core::KeyframeId> window{h.query};
      for (const auto& [k, w] : b.covisibility().covisibles(h.query, 1)) window.push_back(k);
      window_size = static_cast<int>(window.size());
      const Neighbourhood nb = collect(b, h.candidate, 30);
      fs = fuseInto(b, window, nb.points, cam, T_cb, L.match_radius_px, L.max_hamming, cfg_.frontend.min_depth_m);
      for (auto k : window) b.updateCovisibility(k, cfg_.backend.lba_min_covisibility);
      q->loopEdges().insert(h.candidate);
      c->loopEdges().insert(h.query);
      b.bumpCorrectionEpoch();
      b_inertial = b.imuState().initialized;
      ok = true;
      return 0;
    });
    return 0;
  });
  if (!ok) return rep;
  maps_.recordMerge(A->id(), B->id());
  maps_.activate(B->id());   // A is empty now and gets dropped
  if (weld_ba_ && L.weld_ba && !b_inertial) weld_ba_->run(*B, h.query, st, nullptr);
  maps_.events().publish(MapMergedEvent{A->id().value, B->id().value});
  publishCorrections(*B, old_poses, moved_active ? scale : 1.0);
  rep.merged = true;
  rep.window = window_size;
  rep.fused = fs.replaced;
  rep.new_observations = fs.added;
  MVSLAM_INFO("map {} merged into map {} at kf {} -> kf {}: {} keyframes moved ({}), window {} kfs, fused {} points, {} new observations", A->id().value,
              B->id().value, h.query.value, h.candidate.value, moved_kfs, moved_active ? "active map" : "stored map", rep.window, rep.fused, rep.new_observations);
  return rep;
}

}  // namespace mvslam::backend::loop
