#pragma once
// Local-map re-observation at keyframe time (OV2SLAM matchToLocalMap / ORB-SLAM3 SearchByProjection):
// map points seen by the covisible keyframes but not tracked into the new keyframe are projected into
// it and associated with untracked (2-D) keypoints by ORB Hamming distance inside a search radius.
// A match records a *track alias* (dead 2-D track -> existing map point) so the frontend renames its
// live KLT track and the mapper never triangulates a duplicate; verified earlier 2-D observations of
// that track are migrated to the map point.
#include "map/map.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class LocalMapMatcher {
 public:
  LocalMapMatcher(const Config& cfg, const sensors::CameraRig& rig) : cfg_(cfg), rig_(rig) {}
  struct Stats { int candidates = 0, matched = 0, migrated = 0; };
  // Runs inside the caller's write lock; `kf` must already be in the map with its own observations.
  Stats match(map::Map& m, map::KeyFrame& kf, int descriptor_stride) const;

 private:
  Config cfg_;
  const sensors::CameraRig& rig_;
};

int hammingDistance(const std::uint8_t* a, const std::uint8_t* b, int bytes);

}  // namespace mvslam::backend
