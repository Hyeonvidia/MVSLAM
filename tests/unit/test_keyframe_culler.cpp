#include <gtest/gtest.h>

#include "backend/mapper/keyframe_culler.hpp"
#include "map/map_store.hpp"

using namespace mvslam::core;

namespace {
// n keyframes, each observing all `points` map points at keypoint index i.
struct Fixture {
  mvslam::map::MapStore store;
  std::vector<KeyframeId> kfs;
  std::vector<MapPointId> mps;
  Fixture(int n, int points, int observers_per_point) {
    auto m = store.active();
    m->withWriteLock([&](mvslam::map::Map& mm) {
      for (int k = 0; k < n; ++k) {
        auto id = store.keyframeIds().next();
        auto kf = std::make_shared<mvslam::map::KeyFrame>(id, k, SE3(SO3(), Vec3(0.1 * k, 0, 0)));
        kf->keypoints().resize(static_cast<std::size_t>(points));
        kf->mapPointSlots().assign(static_cast<std::size_t>(points), MapPointId{});
        if (!kfs.empty()) { kf->setPrevKf(kfs.back()); mm.keyframe(kfs.back())->setNextKf(id); }
        mm.addKeyFrame(kf);
        kfs.push_back(id);
      }
      for (int i = 0; i < points; ++i) {
        auto id = store.mapPointIds().next();
        mm.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, Vec3(0, 0, 2), kfs.front()));
        for (int k = 0; k < std::min(n, observers_per_point); ++k) mm.addObservation(id, kfs[static_cast<std::size_t>(k)], {i, 0});
        mps.push_back(id);
      }
      for (auto k : kfs) mm.updateCovisibility(k, 1);
      return 0;
    });
  }
};
}  // namespace

TEST(KeyframeCuller, RemovesRedundantMiddleKeyframesOnly) {
  Fixture f(6, 100, 6);  // every point seen by all 6 keyframes -> the middle ones are redundant
  mvslam::Config cfg;
  cfg.backend.lba_min_covisibility = 1;
  mvslam::backend::KeyframeCuller culler(cfg);
  auto m = f.store.active();
  const auto removed = m->withWriteLock([&](mvslam::map::Map& mm) { return culler.cull(mm, f.kfs[5]); });
  EXPECT_EQ(removed.size(), 3u);  // k2, k3, k4
  m->withReadLock([&](const mvslam::map::Map& mm) {
    EXPECT_NE(mm.keyframe(f.kfs[0]), nullptr);  // origin protected
    EXPECT_EQ(mm.keyframe(f.kfs[1]), nullptr);
    EXPECT_EQ(mm.keyframe(f.kfs[2]), nullptr);
    EXPECT_EQ(mm.keyframe(f.kfs[3]), nullptr);
    EXPECT_NE(mm.keyframe(f.kfs[4]), nullptr);  // previous keyframe protected
    EXPECT_NE(mm.keyframe(f.kfs[5]), nullptr);  // latest protected
    EXPECT_TRUE(mm.wasCulled(f.kfs[2]));
    const auto culled = mm.resolvePose(f.kfs[2]);
    EXPECT_TRUE(culled.has_value());
    if (culled) EXPECT_NEAR(culled->translation().x(), 0.2, 1e-12);
    EXPECT_EQ(mm.mapPoint(f.mps[0])->numObservations(), 3);
    EXPECT_EQ(mm.keyframe(f.kfs[4])->prevKf(), f.kfs[0]);  // prev/next chain repaired
    return 0;
  });
}

TEST(KeyframeCuller, CulledPosesFollowTheirLiveReference) {
  Fixture f(6, 100, 6);
  mvslam::Config cfg;
  cfg.backend.lba_min_covisibility = 1;
  mvslam::backend::KeyframeCuller culler(cfg);
  auto m = f.store.active();
  m->withWriteLock([&](mvslam::map::Map& mm) { return culler.cull(mm, f.kfs[5]); });
  m->withWriteLock([&](mvslam::map::Map& mm) {  // backend moves the reference (spanning-tree parent k1) by +1 m
    for (auto k : f.kfs) if (auto kf = mm.keyframe(k)) kf->setT_wb(SE3(SO3(), kf->T_wb().translation() + Vec3(1, 0, 0)));
    return 0;
  });
  m->withReadLock([&](const mvslam::map::Map& mm) {
    for (int i = 1; i <= 3; ++i) {
      const auto T = mm.resolvePose(f.kfs[static_cast<std::size_t>(i)]);
      EXPECT_TRUE(T.has_value());
      if (T) EXPECT_NEAR(T->translation().x(), 1.0 + 0.1 * i, 1e-9) << i;  // correction propagated through the chain
    }
    EXPECT_FALSE(mm.resolvePose(KeyframeId{12345}).has_value());
    return 0;
  });
}

TEST(KeyframeCuller, KeepsKeyframesWhosePointsAreNotRedundant) {
  Fixture f(6, 100, 2);  // each point seen by two keyframes only
  mvslam::Config cfg;
  cfg.backend.lba_min_covisibility = 1;
  mvslam::backend::KeyframeCuller culler(cfg);
  auto m = f.store.active();
  const auto removed = m->withWriteLock([&](mvslam::map::Map& mm) { return culler.cull(mm, f.kfs[5]); });
  EXPECT_TRUE(removed.empty());
}

TEST(KeyframeCuller, DisabledWhenLatestUnknown) {
  Fixture f(3, 30, 3);
  mvslam::Config cfg;
  mvslam::backend::KeyframeCuller culler(cfg);
  auto m = f.store.active();
  EXPECT_TRUE(m->withWriteLock([&](mvslam::map::Map& mm) { return culler.cull(mm, KeyframeId{999}); }).empty());
}
