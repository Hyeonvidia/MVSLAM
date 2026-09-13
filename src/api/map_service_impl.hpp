#pragma once
// MapService implementation over MapStore; commands are executed synchronously in Offline mode and
// queued on the loop worker in Realtime (P1 wires the queue; skeleton executes inline).
#include "map/map_store.hpp"
#include "map/serialization/map_archive.hpp"
#include "mvslam/api/map_service.hpp"

namespace mvslam::api {

class MapServiceImpl final : public MapService {
 public:
  using ArmFn = std::function<void(std::function<void(Result<Pose>)>)>;
  MapServiceImpl(map::MapStore& store, ArmFn arm) : store_(store), arm_(std::move(arm)) {}
  void save(std::string path, std::function<void(Result<void>)> cb) override { cb(map::saveMapStore(store_, path)); }
  void load(std::string path, std::function<void(Result<MapSummary>)> cb) override { cb(map::loadMapStore(store_, path)); }
  void localize(std::optional<Pose>, std::function<void(Result<Pose>)> cb) override {
    if (arm_) arm_(std::move(cb)); else cb(Result<Pose>::failure("localize: relocalization unavailable (no place recognizer / frontend)"));
  }
  Subscription subscribe(EventHandler handler) override { return store_.events().subscribe(std::move(handler)); }
  MapSnapshot snapshot() const override {
    MapSnapshot s;
    auto m = store_.active();
    s.map_id = m->id().value;
    m->withReadLock([&](const map::Map& mm) {
      s.epoch = mm.changeEpoch();
      s.keyframes.reserve(mm.numKeyframes());
      for (const auto& kf : mm.keyframes()) s.keyframes.push_back({kf->id().value, kf->timestamp(), core::fromSE3(kf->T_wb())});
      s.points.reserve(mm.numMapPoints());
      for (const auto& mp : mm.mapPoints()) {
        const auto& X = mp->position();
        s.points.push_back({mp->id().value, static_cast<float>(X.x()), static_cast<float>(X.y()), static_cast<float>(X.z())});
      }
      return 0;
    });
    return s;
  }
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
  ArmFn arm_;
};

}  // namespace mvslam::api
