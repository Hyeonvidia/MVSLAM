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
