#pragma once
// Map: keyframes + map points + covisibility of one session segment. One coarse shared_mutex;
// readers copy out under shared lock, writers hold the unique lock briefly (docs/design/04-map.md §4).
#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "core/ids.hpp"
#include "map/covisibility_graph.hpp"
#include "map/keyframe.hpp"
#include "map/map_point.hpp"

namespace mvslam::map {

struct ImuMapState { bool initialized = false, refined1 = false, refined2 = false; double scale = 1.0; core::SO3 R_wg; };

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
  void removeKeyFrame(core::KeyframeId kf);      // tombstone + detach from graph + anchor migration
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
};

}  // namespace mvslam::map
