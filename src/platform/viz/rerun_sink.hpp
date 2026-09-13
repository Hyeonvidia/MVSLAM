#pragma once
// Rerun viewer sink (Observer over the public API). Logs the camera pose/image/tracks per frame,
// keyframe frustums and the map point cloud from MapService events and snapshots, and the
// keyframe-relative trajectory, plus per-frame/per-keyframe timing series (`stats/**`) and an event
// log (`events`) that show the frontend and the backend workers at work. Zero coupling with the core:
// only mvslam/api headers are used, so the same adapter works against a packaged libmvslam.
// Target: an .rrd file or a live viewer. Layout: viz/mvslam.rbl (Rerun blueprint for app id `mvslam`).
#include <memory>
#include <string>

#include "mvslam/api/system.hpp"

namespace mvslam::viz {

struct RerunOptions {
  std::string target = "mvslam.rrd";   // "*.rrd" -> file; "rerun+http://host:9876/proxy" -> live viewer; "spawn" -> local viewer
  int image_every = 5;                 // camera image + 2-D tracks every N frames (0 = never)
  int points_every_keyframes = 5;      // re-log the map point cloud every N keyframes (0 = only at finish)
  int trajectory_every = 10;           // re-log the full trajectory line strip every N frames (0 = only at finish)
  float point_radius_m = 0.01f;
  std::string application_id = "mvslam";
};

class RerunSink {
 public:
  RerunSink(System& system, RerunOptions options);
  ~RerunSink();
  RerunSink(const RerunSink&) = delete;
  RerunSink& operator=(const RerunSink&) = delete;
  // Call after System::track() for every frame (caller thread). `frame_ms`: wall time of track() (<0 = unknown).
  void onFrame(const FrameInput& input, const TrackResult& result, double frame_ms = -1);
  // Final map + trajectory, then flush. Safe to call once; the destructor calls it too.
  void finish();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace mvslam::viz
