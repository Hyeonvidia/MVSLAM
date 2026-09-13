#pragma once
// Loop closing interfaces (docs/design/03-backend.md §5): verification cascade, correction strategy
// per mode (Sim3 / SE3 / yaw-4DoF), pose-graph optimizer, loose BA. Implementation: P3.
#include <memory>
#include <optional>
#include <vector>

#include "backend/solvers/solver.hpp"
#include "map/map_store.hpp"

namespace mvslam::backend::loop {

struct LoopHypothesis { core::KeyframeId query, candidate; core::MapId candidate_map; core::Sim3 S_query_candidate; int inliers = 0; };

class ILoopVerifier {
 public:
  virtual ~ILoopVerifier() = default;
  virtual std::optional<LoopHypothesis> verify(const map::MapStore& store, core::KeyframeId query, const map::PlaceCandidate& cand) = 0;
};

class ILoopCorrector {  // Sim3Corrector (mono), Se3Corrector (stereo/rgbd), Yaw4DofCorrector (inertial)
 public:
  virtual ~ILoopCorrector() = default;
  virtual void correct(map::MapStore& store, const LoopHypothesis& h) = 0;
};

class PoseGraphOptimizer {
 public:
  explicit PoseGraphOptimizer(std::shared_ptr<graph::ISolver> solver) : solver_(std::move(solver)) {}
  // Essential graph (spanning tree + loop edges + strong covisibility) relative-pose optimization.
  void optimize(map::Map& map, const LoopHypothesis& h, int iterations);
 private:
  std::shared_ptr<graph::ISolver> solver_;
};

}  // namespace mvslam::backend::loop
