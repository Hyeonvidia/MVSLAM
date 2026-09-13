#pragma once
// Atlas-lite: one active map + stored maps, global id generators, single keyframe database and
// event bus shared by all maps. docs/design/04-map.md §3.
#include <memory>
#include <mutex>
#include <vector>

#include "core/ids.hpp"
#include "map/map.hpp"
#include "map/map_events.hpp"
#include "map/place_recognizer.hpp"

namespace mvslam::map {

class MapStore {
 public:
  MapStore() : bus_(MapEventBus::create()) { startNewMap(); }
  std::shared_ptr<Map> active() const { std::lock_guard<std::mutex> lk(mu_); return active_; }
  std::vector<std::shared_ptr<Map>> stored() const { std::lock_guard<std::mutex> lk(mu_); return stored_; }
  std::shared_ptr<Map> startNewMap() {
    std::lock_guard<std::mutex> lk(mu_);
    if (active_) stored_.push_back(active_);
    active_ = std::make_shared<Map>(map_ids_.next());
    return active_;
  }
  // Drops the active map (bootstrap failure) and starts a fresh one; nothing is stored.
  std::shared_ptr<Map> resetActive() {
    std::lock_guard<std::mutex> lk(mu_);
    active_ = std::make_shared<Map>(map_ids_.next());
    return active_;
  }
  // Persistence / relocalization support ------------------------------------------------------
  void addStored(std::shared_ptr<Map> m) { std::lock_guard<std::mutex> lk(mu_); stored_.push_back(std::move(m)); }
  std::shared_ptr<Map> find(core::MapId id) const {
    std::lock_guard<std::mutex> lk(mu_);
    if (active_ && active_->id() == id) return active_;
    for (const auto& m : stored_) if (m->id() == id) return m;
    return nullptr;
  }
  std::vector<std::shared_ptr<Map>> all() const {  // stored maps then the active one
    std::lock_guard<std::mutex> lk(mu_);
    auto v = stored_;
    if (active_) v.push_back(active_);
    return v;
  }
  // Makes a stored map the active one (relocalized into it); the previous active map is stored
  // unless it is empty, in which case it is dropped.
  bool activate(core::MapId id) {
    std::lock_guard<std::mutex> lk(mu_);
    for (auto it = stored_.begin(); it != stored_.end(); ++it) {
      if ((*it)->id() != id) continue;
      auto target = *it;
      stored_.erase(it);
      if (active_ && active_->withReadLock([](const Map& m) { return m.numKeyframes(); }) > 0) stored_.push_back(active_);
      active_ = std::move(target);
      return true;
    }
    return active_ && active_->id() == id;
  }
  core::IdGenerator<core::MapId>& mapIds() { return map_ids_; }
  const core::IdGenerator<core::KeyframeId>& keyframeIds() const { return kf_ids_; }
  const core::IdGenerator<core::MapPointId>& mapPointIds() const { return mp_ids_; }
  const core::IdGenerator<core::TrackId>& trackIds() const { return track_ids_; }
  core::IdGenerator<core::KeyframeId>& keyframeIds() { return kf_ids_; }
  core::IdGenerator<core::MapPointId>& mapPointIds() { return mp_ids_; }
  core::IdGenerator<core::TrackId>& trackIds() { return track_ids_; }
  MapEventBus& events() { return *bus_; }
  void setPlaceRecognizer(std::unique_ptr<IPlaceRecognizer> pr) { recognizer_ = std::move(pr); }
  IPlaceRecognizer* placeRecognizer() { return recognizer_.get(); }

 private:
  mutable std::mutex mu_;
  std::shared_ptr<Map> active_;
  std::vector<std::shared_ptr<Map>> stored_;
  core::IdGenerator<core::MapId> map_ids_;
  core::IdGenerator<core::KeyframeId> kf_ids_;
  core::IdGenerator<core::MapPointId> mp_ids_;
  core::IdGenerator<core::TrackId> track_ids_;
  std::shared_ptr<MapEventBus> bus_;
  std::unique_ptr<IPlaceRecognizer> recognizer_;
};

}  // namespace mvslam::map
