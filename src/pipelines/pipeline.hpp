#pragma once
// Assembled pipeline for one Config (Factory output). Owns the map store, frontend and workers.
#include <memory>

#include "backend/solvers/solver.hpp"
#include "core/thread/worker.hpp"
#include "frontend/visual_frontend.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::pipelines {

struct MapperJob { frontend::KeyframeRequest request; std::vector<core::KeyframeId> merged_from_dropped; };
struct OptimizerJob { core::KeyframeId keyframe; std::vector<core::KeyframeId> joined; };
struct LoopJob { core::KeyframeId keyframe; };

struct Pipeline {
  Config config;
  std::unique_ptr<sensors::CameraRig> rig;
  std::unique_ptr<map::MapStore> maps;
  std::shared_ptr<backend::graph::ISolver> solver;
  std::unique_ptr<frontend::VisualFrontend> frontend;  // null when image code is unavailable (no OpenCV)
  std::unique_ptr<core::Worker<MapperJob>> mapper;
  std::unique_ptr<core::Worker<OptimizerJob>> optimizer;
  std::unique_ptr<core::Worker<LoopJob>> loop;       // null in odometry-only configurations
  bool frontend_available = false;
};

}  // namespace mvslam::pipelines
