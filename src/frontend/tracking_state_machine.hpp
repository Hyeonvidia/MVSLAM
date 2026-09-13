#pragma once
// State pattern for tracking status with sensor-asymmetric recovery (docs/design/02-frontend.md §2.5).
#include <cstdint>

#include "mvslam/api/types.hpp"

namespace mvslam::frontend {

class TrackingStateMachine {
 public:
  struct Params { double recently_lost_inertial_s = 5.0, recently_lost_visual_s = 3.0; int new_map_min_keyframes = 10; };
  enum class Action { None, TryRelocalize, StartNewMap, ResetMap };

  TrackingStateMachine(Params p, bool inertial) : p_(p), inertial_(inertial) {}
  TrackingState state() const { return state_; }
  void onInitialized(Timestamp t) { state_ = TrackingState::Ok; last_ok_ = t; }
  // Feed the per-frame outcome; returns what the pipeline should do next.
  Action onFrame(Timestamp t, bool tracking_ok, int keyframes_in_map);
  void onRelocalized(Timestamp t) { state_ = TrackingState::Ok; last_ok_ = t; }
  void onNewMap() { state_ = TrackingState::NotInitialized; }

 private:
  Params p_;
  bool inertial_;
  TrackingState state_ = TrackingState::NotInitialized;
  Timestamp last_ok_ = 0;
};

}  // namespace mvslam::frontend
