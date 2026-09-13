#pragma once
// Keyframe processing on the mapper worker: depth measurements (stereo / RGB-D), descriptors,
// map point creation (immediate from depth, temporal triangulation for the rest), covisibility,
// map point culling. OpenCV-free: image algorithms are injected strategies.
#include <deque>
#include <memory>
#include <optional>
#include <stop_token>

#include "backend/mapper/local_map_matcher.hpp"
#include "frontend/interfaces.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class Mapper {
 public:
  Mapper(const Config& cfg, const sensors::CameraRig& rig, map::MapStore& maps);
  void setDepthSource(std::unique_ptr<frontend::IDepthSource> d) { depth_ = std::move(d); }
  void setDescriptorExtractor(std::unique_ptr<frontend::IDescriptorExtractor> e) { descriptors_ = std::move(e); }
  // Inserts the keyframe into the active map. Returns an invalid id if the request is stale.
  core::KeyframeId process(frontend::KeyframeRequest req, std::stop_token st);
  struct Stats { int keyframes = 0, points_from_depth = 0, points_triangulated = 0, points_culled = 0, local_matches = 0, migrated_observations = 0; };
  const Stats& stats() const { return stats_; }

 private:
  bool triangulatePending(map::Map& m, const std::vector<map::PendingObservation>& obs, core::Vec3& X_w) const;
  void insertInitialMap(map::Map& m, const frontend::KeyframeRequest& req);
  void cullRecentMapPoints(map::Map& m, core::KeyframeId latest);
  static void setDescriptorFrom(map::MapPoint& mp, const map::KeyFrame& kf, int index, int stride);

  Config cfg_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  std::unique_ptr<frontend::IDepthSource> depth_;
  std::unique_ptr<frontend::IDescriptorExtractor> descriptors_;
  LocalMapMatcher matcher_;
  std::deque<std::pair<core::KeyframeId, core::MapPointId>> recent_points_;  // (created at, id) for culling
  Stats stats_;
};

}  // namespace mvslam::backend
