#include <gtest/gtest.h>

#include "map/map_store.hpp"

using namespace mvslam::core;

TEST(Map, CovisibilityAndAnchorMigration) {
  mvslam::map::MapStore store;
  auto m = store.active();
  KeyframeId k1 = store.keyframeIds().next(), k2 = store.keyframeIds().next(), k3 = store.keyframeIds().next();
  MapPointId p1 = store.mapPointIds().next();
  m->withWriteLock([&](mvslam::map::Map& mm) {
    for (auto k : {k1, k2, k3}) {
      auto kf = std::make_shared<mvslam::map::KeyFrame>(k, 0, SE3());
      kf->keypoints().resize(1);
      mm.addKeyFrame(kf);
    }
    mm.addMapPoint(std::make_shared<mvslam::map::MapPoint>(p1, Vec3(0, 0, 1), k1));
    for (auto k : {k1, k2, k3}) mm.addObservation(p1, k, {0, 0});
    mm.updateCovisibility(k3, 1);
    return 0;
  });
  const auto epoch = m->changeEpoch();
  EXPECT_EQ(epoch, 1u);
  m->withReadLock([&](const mvslam::map::Map& mm) {
    EXPECT_EQ(mm.covisibility().weight(k3, k1), 1);
    EXPECT_EQ(mm.mapPoint(p1)->anchor(), k1);
    EXPECT_EQ(mm.keyframe(k3)->parent(), k1);  // first connection attaches to the spanning tree
    return 0;
  });
  m->withWriteLock([&](mvslam::map::Map& mm) { mm.removeKeyFrame(k1); return 0; });
  m->withReadLock([&](const mvslam::map::Map& mm) {
    EXPECT_FALSE(mm.keyframe(k1));
    EXPECT_EQ(mm.mapPoint(p1)->numObservations(), 2);
    EXPECT_EQ(mm.mapPoint(p1)->anchor(), k2);   // anchor migrates to the oldest remaining observer
    EXPECT_EQ(mm.covisibility().weight(k3, k1), 0);
    return 0;
  });
}

TEST(MapStore, NewMapKeepsGlobalIds) {
  mvslam::map::MapStore store;
  auto first = store.active();
  auto k = store.keyframeIds().next();
  auto second = store.startNewMap();
  EXPECT_NE(first->id(), second->id());
  EXPECT_EQ(store.stored().size(), 1u);
  EXPECT_LT(k, store.keyframeIds().next());
}

TEST(MapEventBus, SubscribeAndUnsubscribe) {
  mvslam::map::MapStore store;
  int count = 0;
  {
    auto sub = store.events().subscribe([&](const mvslam::MapEvent&) { ++count; });
    store.events().publish(mvslam::MapResetEvent{1});
  }
  store.events().publish(mvslam::MapResetEvent{1});
  EXPECT_EQ(count, 1);
}
