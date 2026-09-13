#include <gtest/gtest.h>

#include "frontend/keyframe_policy.hpp"

using namespace mvslam;
using namespace mvslam::frontend;

static Frame frameWith(int n3d, int n2d, int occupied) {
  Frame f;
  for (int i = 0; i < n3d; ++i) { Track t; t.is3d = true; f.tracks.push_back(t); }
  for (int i = 0; i < n2d; ++i) f.tracks.push_back(Track{});
  f.occupied_cells = occupied;
  return f;
}

TEST(KeyframePolicy, ParallaxTriggersAndBackpressureSuppresses) {
  FrontendConfig c;
  ParallaxTrackRatioPolicy p(c);
  Frame kf = frameWith(170, 50, 200), cur = frameWith(160, 50, 190);
  IKeyframePolicy::Context ctx;
  ctx.cur = &cur; ctx.last_kf = &kf; ctx.max_tracks = 300; ctx.frames_since_kf = 10;
  ctx.median_parallax_px = 25;
  EXPECT_TRUE(p.needKeyframe(ctx));
  ctx.median_parallax_px = 5;
  EXPECT_FALSE(p.needKeyframe(ctx));            // below the half-parallax gate
  ctx.median_parallax_px = 25; ctx.optimizer_busy = true;  // 160 > 0.5*300 and backend busy -> suppressed
  EXPECT_FALSE(p.needKeyframe(ctx));
}

TEST(KeyframePolicy, StarvationForcesKeyframe) {
  FrontendConfig c;
  ParallaxTrackRatioPolicy p(c);
  Frame kf = frameWith(150, 50, 200), cur = frameWith(10, 5, 15);
  IKeyframePolicy::Context ctx;
  ctx.cur = &cur; ctx.last_kf = &kf; ctx.max_tracks = 300; ctx.frames_since_kf = 3; ctx.median_parallax_px = 0;
  EXPECT_TRUE(p.needKeyframe(ctx));  // < 20 3D tracks
}

TEST(KeyframePolicy, DecoratorsAddTimeRules) {
  FrontendConfig c;
  auto inertial = std::make_unique<InertialTimeRulePolicy>(std::make_unique<ParallaxTrackRatioPolicy>(c), c);
  Frame kf = frameWith(150, 50, 200), cur = frameWith(150, 50, 200);
  IKeyframePolicy::Context ctx;
  ctx.cur = &cur; ctx.last_kf = &kf; ctx.max_tracks = 300; ctx.frames_since_kf = 3; ctx.median_parallax_px = 0;
  ctx.seconds_since_kf = 0.3; ctx.imu_initialized = false;   // pre-init: 0.25 s rule
  EXPECT_TRUE(inertial->needKeyframe(ctx));
  ctx.imu_initialized = true;                                 // post-init: 0.5 s rule
  EXPECT_FALSE(inertial->needKeyframe(ctx));
  ClosePointRulePolicy stereo(std::make_unique<ParallaxTrackRatioPolicy>(c), c);
  ctx.seconds_since_kf = 1.5;
  EXPECT_TRUE(stereo.needKeyframe(ctx));                      // stereo 1 s force
}
