#pragma once
// Essential-graph pose-graph optimization (ORB-SLAM3 OptimizeEssentialGraph / OV2SLAM localPoseGraph
// on SE3): nodes = keyframes, edges = spanning tree, loop edges, strong covisibility (>= strong_weight),
// inertial prev/next chain; measurements from the pre-correction relative poses except around the
// corrected window; the loop keyframe is fixed. Map points follow their anchor keyframe's correction.
#include <memory>
#include <unordered_map>
#include <vector>

#include "backend/solvers/solver.hpp"
#include "map/map.hpp"

namespace mvslam::backend::loop {

struct PgoOptions { int iterations = 20; int strong_weight = 100; double loop_edge_info = 1.0; bool inertial_chain = false; };

struct PgoReport { bool applied = false; int nodes = 0, edges = 0; double initial_chi2 = 0, final_chi2 = 0; std::vector<std::pair<core::KeyframeId, std::pair<core::SE3, core::SE3>>> corrections; };

class PoseGraphOptimizer {
 public:
  explicit PoseGraphOptimizer(std::shared_ptr<graph::ISolver> solver) : solver_(std::move(solver)) {}
  // Relative measurements of spanning-tree / covisibility / chain edges come from `reference_poses` (the
  // poses before the loop correction was applied to the query window; keyframes missing from it use their
  // current pose). Loop edges (`loop_edges` plus the keyframes' stored loop edges) are measured with the
  // current, corrected poses. The optimization starts from the current poses; `fixed` keyframes (the loop
  // keyframe) stay constant.
  PgoReport optimize(map::Map& map, const std::unordered_map<core::KeyframeId, core::SE3>& reference_poses,
                     const std::vector<core::KeyframeId>& fixed,
                     const std::vector<std::pair<core::KeyframeId, core::KeyframeId>>& loop_edges, const PgoOptions& opt);

 private:
  std::shared_ptr<graph::ISolver> solver_;
};

}  // namespace mvslam::backend::loop
