#pragma once
// Global relocalization of one image against one or more maps (docs/design/02-frontend.md §2.5):
// corners + ORB descriptors on the query image (LOST-time only) -> place recognizer candidates ->
// descriptor matching against the candidate keyframe's local map points -> P3P-RANSAC ->
// MotionOnlyBa -> projection expansion over the local map -> re-optimization. Pure query: maps are
// only read; the caller adopts the result (pose, inlier 2D-3D tracks, reference keyframe).
#include <memory>
#include <optional>
#include <vector>

#include "frontend/interfaces.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend {

struct RelocalizationResult {
  core::SE3 T_wb;
  core::MapId map;
  core::KeyframeId keyframe;   // best candidate keyframe (reference for the frame's T_kb)
  core::SE3 T_wk;              // its pose
  std::vector<Track> tracks;   // inlier 2-D/3-D tracks (ids == map point ids)
  int inliers = 0;
};

class Relocalizer {
 public:
  Relocalizer(const FrontendConfig& c, const sensors::CameraRig& rig, map::MapStore& maps, IFeatureDetector& detector,
              IDescriptorExtractor& descriptors, IRobustPoseSolver& p3p, IPoseOptimizer& pose_optimizer)
      : c_(c), rig_(rig), maps_(maps), detector_(detector), descriptors_(descriptors), p3p_(p3p), pose_optimizer_(pose_optimizer) {}
  std::optional<RelocalizationResult> relocalize(const ImageState& image, const ImageView& raw,
                                                 const std::vector<std::shared_ptr<map::Map>>& maps, int max_candidates = 5);
  struct Stats { int keypoints = 0, candidates = 0, matches = 0, inliers = 0; };
  const Stats& lastStats() const { return stats_; }

 private:
  FrontendConfig c_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  IFeatureDetector& detector_;
  IDescriptorExtractor& descriptors_;
  IRobustPoseSolver& p3p_;
  IPoseOptimizer& pose_optimizer_;
  Stats stats_;
};

}  // namespace mvslam::frontend
