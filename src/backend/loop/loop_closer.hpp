#pragma once
// Loop closing and map merging (docs/design/03-backend.md §5, docs/design/04-map.md §3).
//  LoopDetector: place-recognizer candidates (query's covisible and recent keyframes excluded) -> ORB
//    matching of the query keyframe against the candidate's local map -> P3P-RANSAC -> motion-only BA ->
//    projection expansion -> spatial votes (the query's covisible keyframes must see the candidate's points
//    under the same transform) + temporal carry-over -> inertial gate (gravity-aligned maps agree on
//    roll/pitch). The transform is SE3 for stereo / RGB-D / inertial maps and Sim3 for monocular maps.
//  LoopCloser: same map -> window transform, point fusion (track aliases), loop edges, essential-graph
//    PGO (loop keyframe fixed), weld BA; other map -> rigid snap of the active map onto the stored one
//    (the gravity-aligned frame wins), fusion, activation of the merged map. Events: PosesCorrected,
//    LoopClosed, MapMerged, MapPointsUpdated (published outside the locks).
#include <memory>
#include <optional>
#include <stop_token>
#include <unordered_map>
#include <vector>

#include "backend/local_ba/local_bundle_adjuster.hpp"
#include "backend/loop/pose_graph_optimizer.hpp"
#include "frontend/interfaces.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend::loop {

struct LoopHypothesis {
  core::KeyframeId query, candidate;
  core::MapId query_map, candidate_map;
  core::Sim3 S_cq;                  // candidate-world <- query-world (scale 1 except for monocular maps)
  int matches = 0, inliers = 0, votes = 0;
  std::vector<std::pair<int, core::MapPointId>> inlier_matches;   // query keypoint index -> candidate map point
};

class LoopDetector {
 public:
  LoopDetector(const Config& cfg, const sensors::CameraRig& rig, map::MapStore& maps, frontend::IRobustPoseSolver& p3p,
               frontend::IPoseOptimizer& pose_optimizer)
      : cfg_(cfg), rig_(rig), maps_(maps), p3p_(p3p), pose_optimizer_(pose_optimizer) {}
  // Runs on the loop worker after the keyframe was inserted (descriptors and covisibility available).
  std::optional<LoopHypothesis> detect(core::KeyframeId query);
  // A loop / merge was applied at `query`: cooldown and forget the temporal memory.
  void closed(core::KeyframeId query);
  struct Stats { int queries = 0, candidates = 0, verified = 0, accepted = 0; };
  const Stats& stats() const { return stats_; }

 private:
  Config cfg_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  frontend::IRobustPoseSolver& p3p_;
  frontend::IPoseOptimizer& pose_optimizer_;
  struct Memory { bool valid = false; core::KeyframeId candidate; core::MapId map; std::vector<core::KeyframeId> neighbourhood; int votes = 0, misses = 0; };
  Memory memory_;
  std::uint64_t cooldown_until_ = 0;
  Stats stats_;
};

class LoopCloser {
 public:
  // `weld_ba`: visual local BA run around the query after the correction (null: none).
  LoopCloser(const Config& cfg, const sensors::CameraRig& rig, map::MapStore& maps, std::shared_ptr<graph::ISolver> solver, LocalBundleAdjuster* weld_ba)
      : cfg_(cfg), rig_(rig), maps_(maps), pgo_(std::move(solver)), weld_ba_(weld_ba) {}
  struct Report { bool closed = false, merged = false; int window = 0, fused = 0, new_observations = 0, loop_edges = 0; PgoReport pgo; core::MapId map; };
  Report close(const LoopHypothesis& h, std::stop_token st);

 private:
  Report correctLoop(const LoopHypothesis& h, std::stop_token st);
  Report merge(const LoopHypothesis& h, std::stop_token st);
  void publishCorrections(map::Map& map, const std::unordered_map<core::KeyframeId, core::SE3>& before, double scale);

  Config cfg_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  PoseGraphOptimizer pgo_;
  LocalBundleAdjuster* weld_ba_;
};

}  // namespace mvslam::backend::loop
