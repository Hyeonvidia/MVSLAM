#include <gtest/gtest.h>

#include <random>

#include "backend/mapper/local_map_matcher.hpp"
#include "map/map_store.hpp"

using namespace mvslam::core;

namespace {
constexpr int kStride = 32;
std::vector<std::uint8_t> randomDescriptor(std::mt19937& rng) {
  std::vector<std::uint8_t> d(kStride);
  std::uniform_int_distribution<int> b(1, 255);
  for (auto& x : d) x = static_cast<std::uint8_t>(b(rng));
  return d;
}
void flipBits(std::vector<std::uint8_t>& d, int n, std::mt19937& rng) {
  std::uniform_int_distribution<int> pos(0, kStride * 8 - 1);
  for (int i = 0; i < n; ++i) { const int p = pos(rng); d[static_cast<std::size_t>(p / 8)] ^= static_cast<std::uint8_t>(1 << (p % 8)); }
}
}  // namespace

TEST(LocalMapMatcher, HammingDistance) {
  std::vector<std::uint8_t> a(32, 0), b(32, 0);
  b[0] = 0x0F; b[31] = 0x80;
  EXPECT_EQ(mvslam::backend::hammingDistance(a.data(), b.data(), 32), 5);
  EXPECT_EQ(mvslam::backend::hammingDistance(a.data(), a.data(), 32), 0);
}

TEST(LocalMapMatcher, AssociatesUntrackedKeypointsAndMigratesPendingObservations) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig cc; cc.width = 752; cc.height = 480; cc.intrinsics = {458.654, 457.296, 367.215, 248.375};
  rc.cameras.push_back(cc);
  mvslam::sensors::CameraRig rig(rc);
  const auto& cam = rig.camera(0);
  mvslam::Config cfg;
  cfg.backend.local_match_radius_px = 5.0;
  cfg.backend.local_match_max_hamming = 50;
  mvslam::backend::LocalMapMatcher matcher(cfg, rig);

  std::mt19937 rng(7);
  std::uniform_real_distribution<double> u(-1, 1);
  mvslam::map::MapStore store;
  auto m = store.active();
  const SE3 T_a, T_b(SO3(), Vec3(0.15, 0.02, 0)), T_c(SO3(), Vec3(0.08, 0, 0));  // A (old), C (older pending), B (new)
  const int n = 50;
  std::vector<Vec3> X;
  std::vector<std::vector<std::uint8_t>> desc;
  for (int i = 0; i < n; ++i) { X.emplace_back(1.5 * u(rng), 1.0 * u(rng), 3.5 + u(rng)); desc.push_back(randomDescriptor(rng)); }

  const auto ka = store.keyframeIds().next(), kc = store.keyframeIds().next(), kb = store.keyframeIds().next();
  std::vector<MapPointId> mps;
  std::vector<TrackId> b_tracks;
  m->withWriteLock([&](mvslam::map::Map& mm) {
    // A: observes all points (tracked), descriptors known.
    auto a = std::make_shared<mvslam::map::KeyFrame>(ka, 0, T_a);
    for (int i = 0; i < n; ++i) {
      mvslam::map::Keypoint kp; kp.uv = kp.uv_raw = cam.project(T_a.inverse() * X[static_cast<std::size_t>(i)]); kp.track = store.trackIds().next();
      a->keypoints().push_back(kp);
      a->descriptors().insert(a->descriptors().end(), desc[static_cast<std::size_t>(i)].begin(), desc[static_cast<std::size_t>(i)].end());
    }
    a->mapPointSlots().assign(static_cast<std::size_t>(n), MapPointId{});
    mm.addKeyFrame(a);
    for (int i = 0; i < n; ++i) {
      const MapPointId id{a->keypoints()[static_cast<std::size_t>(i)].track.value};
      auto mp = mm.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, X[static_cast<std::size_t>(i)], ka));
      mp->setDescriptor(desc[static_cast<std::size_t>(i)]);
      mm.addObservation(id, ka, {i, 0});
      mps.push_back(id);
    }
    // C: an intermediate keyframe holding pending 2-D observations of B's (new) tracks.
    auto c = std::make_shared<mvslam::map::KeyFrame>(kc, 1, T_c);
    // B: the new keyframe, same points re-detected as fresh 2-D tracks (+ distractors), slightly noisy descriptors.
    auto b = std::make_shared<mvslam::map::KeyFrame>(kb, 2, T_b);
    for (int i = 0; i < n; ++i) {
      mvslam::map::Keypoint kp;
      kp.uv = kp.uv_raw = cam.project(T_b.inverse() * X[static_cast<std::size_t>(i)]) + Vec2(0.5 * u(rng), 0.5 * u(rng));
      kp.track = store.trackIds().next();
      b_tracks.push_back(kp.track);
      b->keypoints().push_back(kp);
      auto d = desc[static_cast<std::size_t>(i)];
      flipBits(d, 8, rng);
      b->descriptors().insert(b->descriptors().end(), d.begin(), d.end());
      mvslam::map::Keypoint kpc; kpc.uv = kpc.uv_raw = cam.project(T_c.inverse() * X[static_cast<std::size_t>(i)]); kpc.track = kp.track;
      c->keypoints().push_back(kpc);
      mm.pending2d()[kp.track].push_back({kc, i});
    }
    for (int i = 0; i < 20; ++i) {  // distractors: random position, random descriptor
      mvslam::map::Keypoint kp; kp.uv = kp.uv_raw = Vec2(376 + 300 * u(rng), 240 + 200 * u(rng)); kp.track = store.trackIds().next();
      b->keypoints().push_back(kp);
      const auto d = randomDescriptor(rng);
      b->descriptors().insert(b->descriptors().end(), d.begin(), d.end());
    }
    c->mapPointSlots().assign(c->keypoints().size(), MapPointId{});
    b->mapPointSlots().assign(b->keypoints().size(), MapPointId{});
    c->setPrevKf(ka); b->setPrevKf(kc);
    mm.addKeyFrame(c);
    mm.addKeyFrame(b);
    const auto st = matcher.match(mm, *b, kStride);
    EXPECT_EQ(st.candidates, n);
    EXPECT_GE(st.matched, n - 2);
    EXPECT_GE(st.migrated, n - 2);
    return 0;
  });
  m->withReadLock([&](const mvslam::map::Map& mm) {
    int aliased = 0, observed_by_b = 0, observed_by_c = 0;
    for (int i = 0; i < n; ++i) {
      if (mm.hasTrackAlias(b_tracks[static_cast<std::size_t>(i)]) && mm.resolveTrack(b_tracks[static_cast<std::size_t>(i)]) == mps[static_cast<std::size_t>(i)]) ++aliased;
      const auto& obs = mm.mapPoint(mps[static_cast<std::size_t>(i)])->observations();
      observed_by_b += obs.count(kb) ? 1 : 0;
      observed_by_c += obs.count(kc) ? 1 : 0;
    }
    EXPECT_GE(aliased, n - 2);
    EXPECT_GE(observed_by_b, n - 2);
    EXPECT_GE(observed_by_c, n - 2);
    EXPECT_TRUE(mm.pending2d().empty() || static_cast<int>(mm.pending2d().size()) <= 2);
    const auto b = mm.keyframe(kb);
    int slots = 0;
    for (std::size_t i = 0; i < static_cast<std::size_t>(n); ++i) if (b->mapPointSlots()[i].valid()) ++slots;
    EXPECT_GE(slots, n - 2);
    for (std::size_t i = static_cast<std::size_t>(n); i < b->mapPointSlots().size(); ++i) EXPECT_FALSE(b->mapPointSlots()[i].valid());  // distractors stay 2-D
    return 0;
  });
}

TEST(Map, TrackAliasChainsResolve) {
  mvslam::map::MapStore store;
  auto m = store.active();
  m->withWriteLock([&](mvslam::map::Map& mm) {
    mm.addTrackAlias(TrackId{5}, MapPointId{3});
    mm.addTrackAlias(TrackId{9}, MapPointId{5});   // 9 -> 5 -> 3 collapses to 3
    EXPECT_EQ(mm.resolveTrack(TrackId{9}).value, 3u);
    EXPECT_EQ(mm.resolveTrack(TrackId{5}).value, 3u);
    EXPECT_EQ(mm.resolveTrack(TrackId{42}).value, 42u);  // identity without alias
    mm.addTrackAlias(TrackId{3}, MapPointId{3});   // self alias ignored
    EXPECT_FALSE(mm.hasTrackAlias(TrackId{3}));
    return 0;
  });
}
