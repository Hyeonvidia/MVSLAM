#pragma once
// Keyframe culling (ORB-SLAM3 rule): a covisible keyframe is redundant when more than `redundancy`
// of its map points are observed by at least three other keyframes. Keyframes from the previous one
// onwards are protected (the frontend references them), map origins too. A culled keyframe leaves a
// pose record relative to a live reference (Map::CulledRecord) so System::trajectory() keeps following
// backend corrections for frames that were expressed against it.
#include <vector>

#include "map/map.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class KeyframeCuller {
 public:
  explicit KeyframeCuller(const Config& cfg, const sensors::CameraRig* rig = nullptr) : cfg_(cfg), rig_(rig) {}
  // Runs inside the caller's write lock. Returns the ids removed.
  std::vector<core::KeyframeId> cull(map::Map& m, core::KeyframeId latest) const;
 private:
  Config cfg_;
  const sensors::CameraRig* rig_;
};

}  // namespace mvslam::backend
