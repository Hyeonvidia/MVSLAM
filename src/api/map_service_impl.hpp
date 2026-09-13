#pragma once
// MapService implementation over MapStore; commands are executed synchronously in Offline mode and
// queued on the loop worker in Realtime (P1 wires the queue; skeleton executes inline).
#include "map/map_store.hpp"
#include "map/serialization/map_archive.hpp"
#include "mvslam/api/map_service.hpp"

namespace mvslam::api {

class MapServiceImpl final : public MapService {
 public:
  explicit MapServiceImpl(map::MapStore& store) : store_(store) {}
  void save(std::string path, std::function<void(Result<void>)> cb) override { cb(map::saveMapStore(store_, path)); }
  void load(std::string path, std::function<void(Result<MapSummary>)> cb) override { cb(map::loadMapStore(store_, path)); }
  void localize(std::optional<Pose>, std::function<void(Result<Pose>)> cb) override { cb(Result<Pose>::failure("localize: not implemented (P1)")); }
  Subscription subscribe(EventHandler handler) override { return store_.events().subscribe(std::move(handler)); }
  MapSummary summary() const override {
    MapSummary s;
    s.maps = static_cast<int>(store_.stored().size()) + 1;
    auto m = store_.active();
    s.active_map = m->id().value;
    m->withReadLock([&](const map::Map& mm) { s.keyframes = static_cast<int>(mm.numKeyframes()); s.map_points = static_cast<int>(mm.numMapPoints()); return 0; });
    return s;
  }
 private:
  map::MapStore& store_;
};

}  // namespace mvslam::api
