#include <gtest/gtest.h>

#include <random>
#include <unordered_map>

#include "backend/mapper/mapper.hpp"

using namespace mvslam::core;

namespace {
struct FakeDepthSource final : mvslam::frontend::IDepthSource {
  std::unordered_map<std::uint64_t, double> depth_by_track;  // camera-0 depth of tracks that have stereo depth
  std::vector<mvslam::frontend::DepthMeasurement> measure(const mvslam::frontend::KeyframeRequest& req) override {
    std::vector<mvslam::frontend::DepthMeasurement> out(req.frame.tracks.size());
    for (std::size_t i = 0; i < out.size(); ++i) {
      auto it = depth_by_track.find(req.frame.tracks[i].id.value);
      if (it != depth_by_track.end()) out[i].depth = it->second;
    }
    return out;
  }
};

struct Scene {
  mvslam::sensors::CameraRig rig;
  std::vector<Vec3> X;
  explicit Scene(const mvslam::RigConfig& rc) : rig(rc) {}
  mvslam::frontend::KeyframeRequest request(KeyframeId kf, const SE3& T_wb, const std::vector<std::pair<TrackId, std::size_t>>& tracks, bool initial) const {
    mvslam::frontend::KeyframeRequest r;
    r.kf_id = kf;
    r.initial = initial;
    r.frame.T_wb = T_wb;
    for (const auto& [id, pi] : tracks) {
      mvslam::frontend::Track t;
      t.id = id;
      t.uv = t.uv_raw = rig.camera(0).project(rig.T_cb(0) * (T_wb.inverse() * X[pi]));
      r.frame.tracks.push_back(t);
    }
    return r;
  }
};
}  // namespace

TEST(Mapper, DepthPointsObservationsAndTemporalTriangulation) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 752; c0.height = 480; c0.intrinsics = {458.654, 457.296, 367.215, 248.375};
  mvslam::CameraConfig c1 = c0; c1.T_bc = {1, 0, 0, 0.11, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  rc.cameras = {c0, c1};
  Scene scene(rc);
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1, 1);
  for (int i = 0; i < 60; ++i) scene.X.emplace_back(1.5 * u(rng), 1.0 * u(rng), 4.0 + 1.5 * u(rng));

  mvslam::Config cfg;
  cfg.sensor = mvslam::SensorMode::Stereo;
  cfg.backend.lba_min_covisibility = 15;
  mvslam::map::MapStore store;
  mvslam::backend::Mapper mapper(cfg, scene.rig, store);
  auto depth = std::make_unique<FakeDepthSource>();
  std::vector<TrackId> ids;
  for (int i = 0; i < 60; ++i) ids.push_back(store.trackIds().next());
  for (int i = 0; i < 40; ++i) depth->depth_by_track[ids[static_cast<std::size_t>(i)].value] = scene.X[static_cast<std::size_t>(i)].z();  // T_wb = I for KF1
  mapper.setDepthSource(std::move(depth));

  auto tracksOf = [&](int from, int to) { std::vector<std::pair<TrackId, std::size_t>> v; for (int i = from; i < to; ++i) v.emplace_back(ids[static_cast<std::size_t>(i)], static_cast<std::size_t>(i)); return v; };
  const auto k1 = store.keyframeIds().next(), k2 = store.keyframeIds().next(), k3 = store.keyframeIds().next();
  const SE3 T1, T2(SO3(), Vec3(0.2, 0, 0)), T3(SO3(), Vec3(0.4, 0.05, 0));

  ASSERT_TRUE(mapper.process(scene.request(k1, T1, tracksOf(0, 40), true), std::stop_token{}).valid());
  EXPECT_EQ(mapper.stats().points_from_depth, 40);
  // KF2 re-observes the 40 depth points and introduces 20 new 2-D tracks (no depth).
  auto r2 = scene.request(k2, T2, tracksOf(0, 60), false);
  ASSERT_TRUE(mapper.process(r2, std::stop_token{}).valid());
  EXPECT_EQ(mapper.stats().points_triangulated, 0);  // one 2-D observation is not enough
  // KF3: the 20 pending tracks get a second view with ~3 deg parallax -> triangulated.
  ASSERT_TRUE(mapper.process(scene.request(k3, T3, tracksOf(0, 60), false), std::stop_token{}).valid());
  EXPECT_EQ(mapper.stats().points_triangulated, 20);
  EXPECT_EQ(mapper.stats().keyframes, 3);

  store.active()->withReadLock([&](const mvslam::map::Map& m) {
    EXPECT_EQ(m.numKeyframes(), 3u);
    EXPECT_EQ(m.numMapPoints(), 60u);
    for (int i = 0; i < 60; ++i) {
      auto mp = m.mapPoint(MapPointId{ids[static_cast<std::size_t>(i)].value});
      EXPECT_NE(mp, nullptr) << i;
      if (!mp) continue;
      EXPECT_LT((mp->position() - scene.X[static_cast<std::size_t>(i)]).norm(), 1e-6) << i;
      EXPECT_EQ(mp->numObservations(), i < 40 ? 3 : 2) << i;
      EXPECT_EQ(mp->anchor(), i < 40 ? k1 : k2) << i;  // anchor = first observer
    }
    EXPECT_EQ(m.covisibility().weight(k3, k2), 60);
    EXPECT_EQ(m.covisibility().weight(k3, k1), 40);
    EXPECT_EQ(m.keyframe(k3)->parent(), k2);
    EXPECT_EQ(m.keyframe(k3)->prevKf(), k2);
    EXPECT_EQ(m.lastKeyframe(), k3);
    EXPECT_TRUE(m.pending2d().empty());
    return 0;
  });
}

TEST(Mapper, StaleRequestAfterResetIsIgnored) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 752; c0.height = 480; c0.intrinsics = {458.654, 457.296, 367.215, 248.375};
  rc.cameras = {c0};
  Scene scene(rc);
  scene.X.emplace_back(0, 0, 3);
  mvslam::Config cfg;
  mvslam::map::MapStore store;
  mvslam::backend::Mapper mapper(cfg, scene.rig, store);
  const auto k = store.keyframeIds().next();
  auto req = scene.request(k, SE3(), {{store.trackIds().next(), 0}}, false);  // not initial, map empty -> stale
  EXPECT_FALSE(mapper.process(req, std::stop_token{}).valid());
  EXPECT_EQ(mapper.stats().keyframes, 0);
}
