#include "frontend/tracking_state_machine.hpp"

namespace mvslam::frontend {

TrackingStateMachine::Action TrackingStateMachine::onFrame(Timestamp t, bool ok, int keyframes_in_map) {
  switch (state_) {
    case TrackingState::NotInitialized:
      return Action::None;
    case TrackingState::Ok:
      if (ok) { last_ok_ = t; return Action::None; }
      state_ = TrackingState::RecentlyLost;
      return inertial_ ? Action::None : Action::TryRelocalize;  // IMU dead-reckons, visual relocalizes
    case TrackingState::RecentlyLost: {
      if (ok) { state_ = TrackingState::Ok; last_ok_ = t; return Action::None; }
      const double elapsed = static_cast<double>(t - last_ok_) * 1e-9;
      const double limit = inertial_ ? p_.recently_lost_inertial_s : p_.recently_lost_visual_s;
      if (elapsed > limit) {
        state_ = TrackingState::Lost;
        return keyframes_in_map > p_.new_map_min_keyframes ? Action::StartNewMap : Action::ResetMap;
      }
      return inertial_ ? Action::None : Action::TryRelocalize;
    }
    case TrackingState::Lost:
      return Action::None;  // pipeline has already started a new map / reset
  }
  return Action::None;
}

}  // namespace mvslam::frontend
