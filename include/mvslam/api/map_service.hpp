#pragma once
// Map persistence, relocalization commands and map events (Observer). docs/design/04-map.md §5-6.
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "mvslam/api/types.hpp"

namespace mvslam {

struct MapSummary { int maps = 0; std::uint64_t active_map = 0; int keyframes = 0, map_points = 0; };

struct KeyframeInsertedEvent { std::uint64_t map_id, keyframe_id; Timestamp timestamp; Pose T_wb; };
struct KeyframeRemovedEvent { std::uint64_t map_id, keyframe_id; };
struct MapPointsUpdatedEvent { std::uint64_t map_id; std::uint64_t epoch; };
struct PoseCorrection { std::uint64_t keyframe_id; Pose T_wb_old, T_wb_new; };
struct PosesCorrectedEvent { std::uint64_t map_id; std::vector<PoseCorrection> corrections; };
struct MapMergedEvent { std::uint64_t from_map, into_map; };
struct MapResetEvent { std::uint64_t map_id; };
struct LoopClosedEvent { std::uint64_t map_id, keyframe_a, keyframe_b; };
struct ImuInitializedEvent { std::uint64_t map_id; double scale; };

using MapEvent = std::variant<KeyframeInsertedEvent, KeyframeRemovedEvent, MapPointsUpdatedEvent,
                              PosesCorrectedEvent, MapMergedEvent, MapResetEvent, LoopClosedEvent,
                              ImuInitializedEvent>;
using EventHandler = std::function<void(const MapEvent&)>;

class Subscription {  // RAII handle; destroying it unsubscribes.
 public:
  Subscription() = default;
  explicit Subscription(std::function<void()> release) : release_(std::move(release)) {}
  ~Subscription() { if (release_) release_(); }
  Subscription(Subscription&&) noexcept = default;
  Subscription& operator=(Subscription&&) noexcept = default;
  Subscription(const Subscription&) = delete;
  Subscription& operator=(const Subscription&) = delete;
 private:
  std::function<void()> release_;
};

class MapService {
 public:
  virtual ~MapService() = default;
  // Realtime: queued on the loop worker, callback runs on that worker. Offline: synchronous.
  virtual void save(std::string path, std::function<void(Result<void>)> cb) = 0;
  virtual void load(std::string path, std::function<void(Result<MapSummary>)> cb) = 0;
  virtual void localize(std::optional<Pose> prior, std::function<void(Result<Pose>)> cb) = 0;
  // Handlers are invoked on worker threads, outside map locks; they must return quickly.
  virtual Subscription subscribe(EventHandler handler) = 0;
  virtual MapSummary summary() const = 0;
};

}  // namespace mvslam
