#include <gtest/gtest.h>

#include "frontend/tracking_state_machine.hpp"

using namespace mvslam;
using namespace mvslam::frontend;
using A = TrackingStateMachine::Action;

TEST(TrackingStateMachine, VisualRelocalizesThenStartsNewMapAfter3s) {
  TrackingStateMachine sm({5.0, 3.0, 10}, /*inertial=*/false);
  sm.onInitialized(0);
  EXPECT_EQ(sm.onFrame(50'000'000, true, 20), A::None);
  EXPECT_EQ(sm.onFrame(100'000'000, false, 20), A::TryRelocalize);
  EXPECT_EQ(sm.state(), TrackingState::RecentlyLost);
  EXPECT_EQ(sm.onFrame(2'000'000'000, false, 20), A::TryRelocalize);
  EXPECT_EQ(sm.onFrame(3'100'000'000, false, 20), A::StartNewMap);
  EXPECT_EQ(sm.state(), TrackingState::Lost);
}

TEST(TrackingStateMachine, InertialDeadReckons5sAndResetsSmallMaps) {
  TrackingStateMachine sm({5.0, 3.0, 10}, /*inertial=*/true);
  sm.onInitialized(0);
  EXPECT_EQ(sm.onFrame(100'000'000, false, 5), A::None);       // no relocalization attempts with IMU
  EXPECT_EQ(sm.onFrame(4'000'000'000, false, 5), A::None);
  EXPECT_EQ(sm.onFrame(5'100'000'000, false, 5), A::ResetMap); // small map -> reset instead of new map
}

TEST(TrackingStateMachine, RecoversFromRecentlyLost) {
  TrackingStateMachine sm({5.0, 3.0, 10}, false);
  sm.onInitialized(0);
  sm.onFrame(100'000'000, false, 20);
  EXPECT_EQ(sm.onFrame(200'000'000, true, 20), A::None);
  EXPECT_EQ(sm.state(), TrackingState::Ok);
}
