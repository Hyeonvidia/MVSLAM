#pragma once
// Assembled pipeline for one Config (Factory output). Owns the map store, frontend, backend
// workers and the cross-thread preemption signal ("a newer keyframe is waiting").
#include <atomic>
#include <memory>

#include "backend/local_ba/local_bundle_adjuster.hpp"
#include "backend/mapper/keyframe_culler.hpp"
#include "backend/mapper/mapper.hpp"
#include "backend/solvers/solver.hpp"
#include "core/thread/worker.hpp"
#include "frontend/visual_frontend.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::pipelines {

struct MapperJob { frontend::KeyframeRequest request; std::vector<core::KeyframeId> dropped; };
struct OptimizerJob { core::KeyframeId keyframe; std::vector<core::KeyframeId> joined; };
struct LoopJob { core::KeyframeId keyframe; };

struct Pipeline {
  Config config;
  std::unique_ptr<sensors::CameraRig> rig;
  std::unique_ptr<map::MapStore> maps;
  std::shared_ptr<backend::graph::ISolver> solver;
  std::unique_ptr<frontend::VisualFrontend> frontend;   // null when image code is unavailable (no OpenCV)
  std::unique_ptr<backend::Mapper> mapper_logic;
  std::unique_ptr<backend::LocalBundleAdjuster> lba;
  std::unique_ptr<backend::KeyframeCuller> culler;
  std::unique_ptr<core::Worker<MapperJob>> mapper;
  std::unique_ptr<core::Worker<OptimizerJob>> optimizer;
  std::unique_ptr<core::Worker<LoopJob>> loop;          // null in odometry-only configurations
  std::atomic<std::uint64_t> latest_keyframe_submitted{0};  // LBA preemption: stop when a newer KF arrived
  bool frontend_available = false;
};

}  // namespace mvslam::pipelines
