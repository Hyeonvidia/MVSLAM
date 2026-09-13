#pragma once
// Local BA on the optimizer worker: snapshot (read lock) -> library-independent Problem ->
// ISolver (time budget, preemption by a newer keyframe) -> outlier removal / L2 refine ->
// write-back (write lock) + MapPointsUpdated event. docs/design/03-backend.md §2.
#include <functional>
#include <memory>
#include <stop_token>

#include "backend/local_ba/window_selector.hpp"
#include "backend/solvers/solver.hpp"
#include "map/map.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::backend {

class LocalBundleAdjuster {
 public:
  LocalBundleAdjuster(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver);
  struct Report { bool ran = false, applied = false, preempted = false; int keyframes = 0, points = 0, observations = 0, outliers = 0; double initial_chi2 = 0, final_chi2 = 0, time_s = 0; };
  Report run(map::Map& map, core::KeyframeId latest, std::stop_token st, std::function<bool()> newer_keyframe_pending = nullptr);
  const Report& last() const { return last_; }

 private:
  Config cfg_;
  const sensors::CameraRig& rig_;
  std::shared_ptr<graph::ISolver> solver_;
  std::unique_ptr<IWindowSelector> selector_;
  Report last_;
};

}  // namespace mvslam::backend
