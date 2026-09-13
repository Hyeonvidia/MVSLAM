#pragma once
// Map: keyframes + map points + covisibility of one session segment. One coarse shared_mutex;
// readers copy out under shared lock, writers hold the unique lock briefly (docs/design/04-map.md §4).
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "core/ids.hpp"
#include "map/covisibility_graph.hpp"
#include "map/keyframe.hpp"
#include "map/map_point.hpp"

namespace mvslam::map {

struct ImuMapState { bool initialized = false, refined1 = false, refined2 = false; double scale = 1.0; core::SO3 R_wg; };

// Observation of a not-yet-triangulated track in a keyframe (mapper temporal triangulation input).
struct PendingObservation { core::KeyframeId kf; int keypoint_index = -1; };

class Map {
 public:
  explicit Map(core::MapId id) : id_(id) {}
  core::MapId id() const { return id_; }

  // ---- locking ----------------------------------------------------------------------------
  template <typename F> auto withReadLock(F&& f) const { std::shared_lock lk(mu_); return f(*this); }
  template <typename F> auto withWriteLock(F&& f) { std::unique_lock lk(mu_); auto r = f(*this); epoch_.fetch_add(1, std::memory_order_acq_rel); return r; }
  std::uint64_t changeEpoch() const { return epoch_.load(std::memory_order_acquire); }

  // ---- mutation (call only inside withWriteLock) -------------------------------------------
  std::shared_ptr<KeyFrame> addKeyFrame(std::shared_ptr<KeyFrame> kf);
  std::shared_ptr<MapPoint> addMapPoint(std::shared_ptr<MapPoint> mp);
  void addObservation(core::MapPointId mp, core::KeyframeId kf, ObservationSlot slot);
  void removeObservation(core::MapPointId mp, core::KeyframeId kf);   // clears the keyframe slot too
  void removeKeyFrame(core::KeyframeId kf);      // tombstone + detach from graph + anchor migration; points left with < 2 observations are removed
  void removeMapPoint(core::MapPointId mp);
  void updateCovisibility(core::KeyframeId kf, int min_shared = 15);
  void reclaim(std::uint64_t oldest_live_epoch);   // frees tombstones no reader can still hold

  // ---- queries (inside either lock) --------------------------------------------------------
  std::shared_ptr<KeyFrame> keyframe(core::KeyframeId id) const;
  std::shared_ptr<MapPoint> mapPoint(core::MapPointId id) const;
  const CovisibilityGraph& covisibility() const { return cov_; }
  std::vector<std::shared_ptr<KeyFrame>> keyframes() const;
  std::vector<std::shared_ptr<MapPoint>> mapPoints() const;
  std::size_t numKeyframes() const { return keyframes_.size(); }
  std::size_t numMapPoints() const { return map_points_.size(); }
  ImuMapState& imuState() { return imu_state_; }
  const ImuMapState& imuState() const { return imu_state_; }
  const std::vector<core::KeyframeId>& origins() const { return origins_; }
  std::unordered_map<core::TrackId, std::vector<PendingObservation>>& pending2d() { return pending2d_; }
  const std::unordered_map<core::TrackId, std::vector<PendingObservation>>& pending2d() const { return pending2d_; }
  core::KeyframeId lastKeyframe() const { return last_kf_; }
  // Culled keyframes keep their pose relative to a reference keyframe (spanning-tree parent, ORB-SLAM
  // mTcp): T_wc = T_wr * T_rc. References may be culled later themselves, so resolvePose() walks the
  // chain until a live keyframe (map origins are never culled); the absolute pose at culling time is
  // the last resort. Lets System::trajectory() follow backend corrections for frames whose reference
  // keyframe is gone.
  struct CulledRecord { core::KeyframeId reference; core::SE3 T_rc, T_wc_at_cull; };
  void rememberCulled(core::KeyframeId k, CulledRecord r) { culled_[k] = std::move(r); }
  bool wasCulled(core::KeyframeId k) const { return culled_.count(k) != 0; }
  std::optional<core::SE3> resolvePose(core::KeyframeId k) const {
    core::SE3 T_rk;  // reference -> k, accumulated along the chain
    for (int hop = 0; hop < 4096; ++hop) {
      if (auto kf = keyframe(k)) return kf->T_wb() * T_rk;
      auto it = culled_.find(k);
      if (it == culled_.end()) return std::nullopt;
      if (!it->second.reference.valid()) return it->second.T_wc_at_cull * T_rk;
      T_rk = it->second.T_rc * T_rk;
      k = it->second.reference;
    }
    return std::nullopt;
  }
  void setLastKeyframe(core::KeyframeId k) { last_kf_ = k; }
  // Track aliases: a 2-D track matched to an existing map point (local-map matching) maps to that
  // point's id from then on. resolveTrack() follows alias chains; identity when there is none.
  void addTrackAlias(core::TrackId t, core::MapPointId mp) { const auto target = resolveTrack(core::TrackId{mp.value}); if (target.value != t.value) track_aliases_[t] = target; }
  core::MapPointId resolveTrack(core::TrackId t) const {
    core::MapPointId id{t.value};
    for (int hop = 0; hop < 8; ++hop) { auto it = track_aliases_.find(core::TrackId{id.value}); if (it == track_aliases_.end()) break; id = it->second; }
    return id;
  }
  bool hasTrackAlias(core::TrackId t) const { return track_aliases_.count(t) != 0; }

 private:
  core::MapId id_;
  mutable std::shared_mutex mu_;
  std::atomic<std::uint64_t> epoch_{0};
  std::unordered_map<core::KeyframeId, std::shared_ptr<KeyFrame>> keyframes_;
  std::unordered_map<core::MapPointId, std::shared_ptr<MapPoint>> map_points_;
  std::vector<std::shared_ptr<KeyFrame>> kf_tombstones_;
  std::vector<std::shared_ptr<MapPoint>> mp_tombstones_;
  CovisibilityGraph cov_;
  std::vector<core::KeyframeId> origins_;
  ImuMapState imu_state_;
  std::unordered_map<core::TrackId, std::vector<PendingObservation>> pending2d_;
  core::KeyframeId last_kf_;
  std::unordered_map<core::KeyframeId, CulledRecord> culled_;
  std::unordered_map<core::TrackId, core::MapPointId> track_aliases_;
};

}  // namespace mvslam::map
