#include "frontend/relocalizer.hpp"

#include <algorithm>
#include <limits>
#include <set>
#include <unordered_map>

#include "backend/mapper/local_map_matcher.hpp"  // hammingDistance
#include "core/log.hpp"

namespace mvslam::frontend {

namespace {
constexpr int kStride = 32;
struct LocalPoint { core::MapPointId id; core::Vec3 X_w; std::vector<std::uint8_t> desc; };

bool allZero(const std::uint8_t* d) { for (int i = 0; i < kStride; ++i) if (d[i]) return false; return true; }

// Best query keypoint for a map point descriptor (Hamming <= max, ratio 0.8 against the second best).
int bestQuery(const std::vector<std::uint8_t>& mp_desc, const std::vector<std::uint8_t>& q_desc, const std::vector<bool>& q_used,
              int max_hamming, int& best_dist, const std::vector<int>* restrict = nullptr) {
  int best = std::numeric_limits<int>::max(), second = best, best_i = -1;
  auto consider = [&](int i) {
    if (q_used[static_cast<std::size_t>(i)]) return;
    const std::uint8_t* d = q_desc.data() + static_cast<std::size_t>(i) * kStride;
    if (allZero(d)) return;
    const int h = backend::hammingDistance(mp_desc.data(), d, kStride);
    if (h < best) { second = best; best = h; best_i = i; } else if (h < second) { second = h; }
  };
  if (restrict) for (int i : *restrict) consider(i);
  else for (int i = 0; i < static_cast<int>(q_desc.size() / kStride); ++i) consider(i);
  if (best_i < 0 || best > max_hamming || (second < std::numeric_limits<int>::max() && best > 0.8 * second)) return -1;
  best_dist = best;
  return best_i;
}
}  // namespace

std::optional<RelocalizationResult> Relocalizer::relocalize(const ImageState& image, const ImageView& raw,
                                                            const std::vector<std::shared_ptr<map::Map>>& maps, int max_candidates) {
  stats_ = {};
  const auto& cam = rig_.camera(0);
  const auto uv_raw = detector_.detect(image, {}, detector_.cellCount(cam.width(), cam.height()));
  stats_.keypoints = static_cast<int>(uv_raw.size());
  if (uv_raw.size() < 30) return std::nullopt;
  std::vector<core::Vec2> uv;
  uv.reserve(uv_raw.size());
  for (const auto& p : uv_raw) uv.push_back(cam.undistort(p));
  const auto q_desc = descriptors_.describe(raw, uv_raw);
  auto* recognizer = maps_.placeRecognizer();
  if (!recognizer) return std::nullopt;
  const auto candidates = recognizer->query(q_desc, kStride, {}, max_candidates);
  stats_.candidates = static_cast<int>(candidates.size());

  for (const auto& cand : candidates) {
    std::shared_ptr<map::Map> map;
    for (const auto& m : maps) if (m->id() == cand.map) map = m;
    if (!map) continue;
    // Local map of the candidate: its points plus those of its strongest covisible keyframes.
    std::vector<LocalPoint> local;
    core::SE3 T_wk;
    bool have_kf = false;
    map->withReadLock([&](const map::Map& m) {
      auto kf = m.keyframe(cand.keyframe);
      if (!kf || kf->bad()) return 0;
      have_kf = true;
      T_wk = kf->T_wb();
      std::vector<core::KeyframeId> kfs{cand.keyframe};
      int n = 0;
      for (const auto& [k, w] : m.covisibility().covisibles(cand.keyframe, 1)) { if (n++ >= 10) break; kfs.push_back(k); }
      std::set<core::MapPointId> seen;
      for (auto k : kfs) {
        auto lk = m.keyframe(k);
        if (!lk) continue;
        for (auto id : lk->mapPointSlots()) {
          if (!id.valid() || !seen.insert(id).second) continue;
          auto mp = m.mapPoint(id);
          if (!mp || mp->bad() || mp->descriptor().size() < kStride) continue;
          local.push_back({id, mp->position(), mp->descriptor()});
        }
      }
      return 0;
    });
    if (!have_kf || local.size() < 15) continue;

    // 1. Descriptor matches (map point -> query keypoint), unique per keypoint.
    std::vector<bool> used(uv.size(), false);
    struct Match { int q; std::size_t p; int dist; };
    std::unordered_map<int, Match> by_query;
    for (std::size_t p = 0; p < local.size(); ++p) {
      int dist = 0;
      const int q = bestQuery(local[p].desc, q_desc, used, c_.reloc_max_hamming, dist);
      if (q < 0) continue;
      auto it = by_query.find(q);
      if (it == by_query.end() || dist < it->second.dist) by_query[q] = {q, p, dist};
    }
    stats_.matches = std::max(stats_.matches, static_cast<int>(by_query.size()));
    MVSLAM_DEBUG("reloc: candidate kf {} (score {:.2f}) local points {} matches {}", cand.keyframe.value, cand.score, local.size(), by_query.size());
    if (static_cast<int>(by_query.size()) < 15) continue;
    Frame f;
    for (const auto& [q, mt] : by_query) {
      Track t;
      t.id = core::TrackId{local[mt.p].id.value};
      t.uv_raw = uv_raw[static_cast<std::size_t>(q)];
      t.uv = uv[static_cast<std::size_t>(q)];
      t.is3d = true;
      t.X_w = local[mt.p].X_w;
      f.tracks.push_back(t);
      used[static_cast<std::size_t>(q)] = true;
    }
    std::sort(f.tracks.begin(), f.tracks.end(), [](const Track& a, const Track& b) { return a.id < b.id; });  // deterministic order

    // 2. P3P-RANSAC + motion-only BA.
    core::SE3 T;
    std::vector<int> outliers;
    if (!p3p_.solve(f, T, outliers)) { MVSLAM_DEBUG("reloc: p3p failed ({} matches)", f.tracks.size()); continue; }
    f.T_wb = T;
    for (int i : outliers) f.tracks[static_cast<std::size_t>(i)].is3d = false;
    auto po = pose_optimizer_.optimize(f, 0);
    MVSLAM_DEBUG("reloc: p3p outliers {} -> ba inliers {}", outliers.size(), po.inliers);
    if (!po.ok && po.inliers < 15) continue;

    // 3. Projection expansion over the whole local map, then re-optimization.
    const core::SE3 T_cw = rig_.T_cb(0) * f.T_wb.inverse();
    std::set<core::MapPointId> have;
    for (const auto& t : f.tracks) have.insert(core::MapPointId{t.id.value});
    const double radius = 8.0;
    for (const auto& lp : local) {
      if (have.count(lp.id)) continue;
      const core::Vec3 pc = T_cw * lp.X_w;
      if (pc.z() <= c_.min_depth_m) continue;
      const core::Vec2 proj = cam.project(pc);
      if (!cam.inImage(proj, 1)) continue;
      std::vector<int> near;
      for (std::size_t i = 0; i < uv.size(); ++i) if (!used[i] && (uv[i] - proj).norm() <= radius) near.push_back(static_cast<int>(i));
      if (near.empty()) continue;
      int dist = 0;
      const int q = bestQuery(lp.desc, q_desc, used, c_.reloc_max_hamming, dist, &near);
      if (q < 0) continue;
      Track t;
      t.id = core::TrackId{lp.id.value};
      t.uv_raw = uv_raw[static_cast<std::size_t>(q)];
      t.uv = uv[static_cast<std::size_t>(q)];
      t.is3d = true;
      t.X_w = lp.X_w;
      f.tracks.push_back(t);
      used[static_cast<std::size_t>(q)] = true;
    }
    for (auto& t : f.tracks) if (have.count(core::MapPointId{t.id.value})) t.is3d = t.is3d;  // keep BA verdicts of pass 2
    po = pose_optimizer_.optimize(f, 0);
    stats_.inliers = std::max(stats_.inliers, po.inliers);
    MVSLAM_DEBUG("reloc: after expansion tracks {} inliers {}", f.tracks.size(), po.inliers);
    if (po.inliers < c_.reloc_min_inliers) continue;

    RelocalizationResult r;
    r.T_wb = f.T_wb;
    r.map = cand.map;
    r.keyframe = cand.keyframe;
    r.T_wk = T_wk;
    r.inliers = po.inliers;
    for (const auto& t : f.tracks) if (t.is3d) r.tracks.push_back(t);
    MVSLAM_INFO("relocalized: map {} kf {} matches {} inliers {} (candidates {})", cand.map.value, cand.keyframe.value, by_query.size(),
                po.inliers, candidates.size());
    return r;
  }
  MVSLAM_DEBUG("reloc: FAILED keypoints={} candidates={} best_matches={} best_inliers={}", stats_.keypoints, stats_.candidates, stats_.matches, stats_.inliers);
  return std::nullopt;
}

}  // namespace mvslam::frontend
