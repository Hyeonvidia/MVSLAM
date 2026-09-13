#include "backend/mapper/local_map_matcher.hpp"

#include <bit>
#include <cstring>
#include <limits>
#include <set>
#include <unordered_map>

namespace mvslam::backend {

int hammingDistance(const std::uint8_t* a, const std::uint8_t* b, int bytes) {
  int d = 0;
  int i = 0;
  for (; i + 8 <= bytes; i += 8) {
    std::uint64_t x, y;
    std::memcpy(&x, a + i, 8);
    std::memcpy(&y, b + i, 8);
    d += std::popcount(x ^ y);
  }
  for (; i < bytes; ++i) d += std::popcount(static_cast<unsigned>(a[i] ^ b[i]));
  return d;
}

namespace {
bool allZero(const std::uint8_t* d, int n) {
  for (int i = 0; i < n; ++i) if (d[i]) return false;
  return true;
}
}  // namespace

LocalMapMatcher::Stats LocalMapMatcher::match(map::Map& m, map::KeyFrame& kf, int stride) const {
  Stats st;
  const auto& kps = kf.keypoints();
  const auto& desc = kf.descriptors();
  if (stride <= 0 || desc.size() < kps.size() * static_cast<std::size_t>(stride)) return st;
  const auto& bc = cfg_.backend;

  // 1. Local keyframes: covisibles of the new keyframe plus the previous-keyframe chain.
  std::set<core::KeyframeId> local;
  int taken = 0;
  for (const auto& [k, w] : m.covisibility().covisibles(kf.id(), 1)) {
    if (taken++ >= bc.local_match_max_keyframes) break;
    local.insert(k);
  }
  core::KeyframeId p = kf.prevKf();
  for (int i = 0; i < 3 && p.valid(); ++i) {
    auto pk = m.keyframe(p);
    if (!pk) break;
    local.insert(p);
    p = pk->prevKf();
  }
  local.erase(kf.id());

  // 2. Candidate map points: observed by a local keyframe, not by the new one, with a descriptor.
  std::set<core::MapPointId> observed;
  for (auto id : kf.mapPointSlots()) if (id.valid()) observed.insert(id);
  std::set<core::MapPointId> candidates;
  for (auto k : local) {
    auto lk = m.keyframe(k);
    if (!lk) continue;
    for (auto id : lk->mapPointSlots()) if (id.valid() && !observed.count(id)) candidates.insert(id);
  }
  st.candidates = static_cast<int>(candidates.size());
  if (candidates.empty()) return st;

  // 3. Grid of untracked keypoints (cell == search radius) for the radius queries.
  const auto& cam = rig_.camera(0);
  const double r = bc.local_match_radius_px;
  const int ncx = std::max(1, static_cast<int>(std::ceil(cam.width() / r)));
  std::unordered_map<int, std::vector<int>> grid;
  auto cellOf = [&](const core::Vec2& uv) { return static_cast<int>(uv.y() / r) * ncx + static_cast<int>(uv.x() / r); };
  for (std::size_t i = 0; i < kps.size(); ++i) {
    if (kf.mapPointSlots()[i].valid() || kps[i].outlier) continue;
    if (allZero(desc.data() + i * static_cast<std::size_t>(stride), stride)) continue;
    grid[cellOf(kps[i].uv)].push_back(static_cast<int>(i));
  }
  if (grid.empty()) return st;

  // 4. Projection + descriptor search; one best keypoint per map point, one map point per keypoint.
  const core::SE3 T_cw = rig_.T_cb(0) * kf.T_wb().inverse();
  struct Hit { core::MapPointId mp; int dist; };
  std::unordered_map<int, Hit> best_for_keypoint;
  for (auto id : candidates) {
    auto mp = m.mapPoint(id);
    if (!mp || mp->bad() || mp->descriptor().size() < static_cast<std::size_t>(stride)) continue;
    const core::Vec3 pc = T_cw * mp->position();
    if (pc.z() <= cfg_.frontend.min_depth_m || pc.z() > cfg_.frontend.max_depth_m) continue;
    const core::Vec2 uv = cam.project(pc);
    if (!cam.inImage(uv, 1)) continue;
    int best = std::numeric_limits<int>::max(), second = best, best_i = -1;
    const int cx = static_cast<int>(uv.x() / r), cy = static_cast<int>(uv.y() / r);
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        auto it = grid.find((cy + dy) * ncx + (cx + dx));
        if (it == grid.end()) continue;
        for (int i : it->second) {
          if ((kps[static_cast<std::size_t>(i)].uv - uv).norm() > r) continue;
          const int d = hammingDistance(mp->descriptor().data(), desc.data() + static_cast<std::size_t>(i) * static_cast<std::size_t>(stride), stride);
          if (d < best) { second = best; best = d; best_i = i; } else if (d < second) { second = d; }
        }
      }
    }
    if (best_i < 0 || best > bc.local_match_max_hamming) continue;
    if (second < std::numeric_limits<int>::max() && best > 0.8 * second) continue;  // ambiguous
    auto cur = best_for_keypoint.find(best_i);
    if (cur == best_for_keypoint.end() || best < cur->second.dist) best_for_keypoint[best_i] = {id, best};
  }

  // 5. Commit: observation, alias, descriptor refresh, migration of verified pending 2-D observations.
  const double reproj2 = bc.triangulation_max_reproj_px * bc.triangulation_max_reproj_px;
  for (const auto& [i, hit] : best_for_keypoint) {
    auto& kp = kf.keypoints()[static_cast<std::size_t>(i)];
    const core::TrackId old_track = kp.track;
    m.addTrackAlias(old_track, hit.mp);
    kp.track = core::TrackId{hit.mp.value};
    m.addObservation(hit.mp, kf.id(), {i, 0});
    auto mp = m.mapPoint(hit.mp);
    mp->setDescriptor(std::vector<std::uint8_t>(desc.begin() + i * stride, desc.begin() + (i + 1) * stride));
    ++st.matched;
    auto pend = m.pending2d().find(old_track);
    if (pend == m.pending2d().end()) continue;
    for (const auto& o : pend->second) {
      auto okf = m.keyframe(o.kf);
      if (!okf || o.kf == kf.id() || mp->observations().count(o.kf)) continue;
      const auto& okp = okf->keypoints()[static_cast<std::size_t>(o.keypoint_index)];
      const core::Vec3 pc = rig_.T_cb(0) * (okf->T_wb().inverse() * mp->position());
      if (pc.z() <= cfg_.frontend.min_depth_m || (cam.project(pc) - okp.uv).squaredNorm() > reproj2) continue;
      okf->keypoints()[static_cast<std::size_t>(o.keypoint_index)].track = kp.track;
      m.addObservation(hit.mp, o.kf, {o.keypoint_index, 0});
      ++st.migrated;
    }
    m.pending2d().erase(pend);
  }
  return st;
}

}  // namespace mvslam::backend
