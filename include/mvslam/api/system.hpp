#pragma once
// Facade over the whole pipeline (docs/design/05-api-config.md §3). Thread contract:
//  * pushImu() and track() are called from one thread (the caller serializes them);
//  * mapService() operations may be called from any thread (they are commands on a queue).
#include <memory>
#include <string>
#include <vector>

#include "mvslam/api/config.hpp"
#include "mvslam/api/map_service.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam {

struct SystemStats {
  double frontend_ms_avg = 0, mapper_ms_avg = 0, optimizer_ms_avg = 0;
  int mapper_queue = 0, optimizer_queue = 0, loop_queue = 0;
  MapSummary map;
};

class System {
 public:
  explicit System(Config cfg);            // validates config, assembles the pipeline, starts workers (Realtime)
  ~System();                              // stops workers (join), warns about unsaved maps
  System(System&&) noexcept;
  System& operator=(System&&) noexcept;
  System(const System&) = delete;
  System& operator=(const System&) = delete;

  void pushImu(const ImuSample& sample);            // timestamps must be non-decreasing
  TrackResult track(const FrameInput& input);        // non-blocking in Realtime, synchronous in Offline
  MapService& mapService();
  // Stateless single-frame localization against every map (loaded + current); tracking is untouched.
  Result<Pose> localize(const FrameInput& input);
  std::vector<PoseStamped> trajectory() const;       // all frames, expressed through their reference keyframes
  SystemStats stats() const;
  const Config& config() const;
  std::string effectiveConfigYaml() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mvslam
