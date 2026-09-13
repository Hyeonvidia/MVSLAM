#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iterator>

#include "map/hbst_place_recognizer.hpp"
#include "map/map_store.hpp"
#include "map/serialization/map_archive.hpp"

using namespace mvslam::core;

namespace {
std::string readFile(const std::string& p) { std::ifstream f(p, std::ios::binary); return std::string(std::istreambuf_iterator<char>(f), {}); }

// Two maps: the first with 3 keyframes / 5 points / covisibility / alias / culled record, the second (stored) with one keyframe.
void populate(mvslam::map::MapStore& store, std::vector<KeyframeId>& kfs, std::vector<MapPointId>& mps) {
  auto m = store.active();
  m->withWriteLock([&](mvslam::map::Map& mm) {
    for (int k = 0; k < 3; ++k) {
      auto id = store.keyframeIds().next();
      auto kf = std::make_shared<mvslam::map::KeyFrame>(id, 1000 + k, SE3(SO3::exp(Vec3(0.01 * k, 0, 0.02)), Vec3(0.1 * k, 0.2, -0.3)));
      kf->setVelocity(Vec3(1, 2, 3));
      kf->setBias((Vec6() << 1, 2, 3, 4, 5, 6).finished() * 1e-3);
      for (int i = 0; i < 5; ++i) {
        mvslam::map::Keypoint kp;
        kp.uv_raw = Vec2(10 * i + k, 20 * i); kp.uv = kp.uv_raw + Vec2(0.5, -0.5); kp.level = i % 2; kp.cam = 0;
        kp.track = TrackId{static_cast<std::uint64_t>(100 + i)}; kp.depth = 1.5 + i; kp.info = 0.9; kp.has_right = i % 2 == 0; kp.uv_right = Vec2(3, 4); kp.outlier = i == 4;
        kf->keypoints().push_back(kp);
        for (int b = 0; b < 32; ++b) kf->descriptors().push_back(static_cast<std::uint8_t>((i * 37 + b * 11 + k) & 0xFF));
      }
      kf->mapPointSlots().assign(5, MapPointId{});
      if (!kfs.empty()) { kf->setPrevKf(kfs.back()); mm.keyframe(kfs.back())->setNextKf(id); }
      mm.addKeyFrame(kf);
      kfs.push_back(id);
    }
    for (int i = 0; i < 5; ++i) {
      auto id = MapPointId{static_cast<std::uint64_t>(100 + i)};
      auto mp = mm.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, Vec3(i, 2.0 * i, 3.5), kfs[0]));
      mp->setDescriptor(std::vector<std::uint8_t>(32, static_cast<std::uint8_t>(i)));
      mp->setCounters(10 + i, 7 + i);
      for (auto k : kfs) mm.addObservation(id, k, {i, 0});
      mps.push_back(id);
    }
    for (auto k : kfs) mm.updateCovisibility(k, 1);
    mm.addTrackAlias(TrackId{555}, MapPointId{101});
    mm.rememberCulled(KeyframeId{999}, {kfs[1], SE3(SO3(), Vec3(0.5, 0, 0)), SE3(SO3(), Vec3(9, 9, 9))});
    mm.setLastKeyframe(kfs.back());
    mm.imuState().initialized = true; mm.imuState().scale = 1.5; mm.imuState().R_wg = SO3::exp(Vec3(0, 0.1, 0));
    mm.keyframe(kfs[2])->loopEdges().insert(kfs[0]);
    return 0;
  });
  auto second = store.startNewMap();  // previous becomes stored; keep track ids monotonic
  second->withWriteLock([&](mvslam::map::Map& mm) {
    auto id = store.keyframeIds().next();
    auto kf = std::make_shared<mvslam::map::KeyFrame>(id, 5000, SE3());
    kf->keypoints().resize(1);
    kf->mapPointSlots().assign(1, MapPointId{});
    mm.addKeyFrame(kf);
    kfs.push_back(id);
    return 0;
  });
}
}  // namespace

TEST(MapArchive, RoundTripIsByteIdenticalAndRestoresEverything) {
  const std::string path = "test_map_archive_roundtrip.mvs", path2 = "test_map_archive_roundtrip2.mvs";
  mvslam::map::MapStore a;
  std::vector<KeyframeId> kfs;
  std::vector<MapPointId> mps;
  populate(a, kfs, mps);
  const auto saved = mvslam::map::saveMapStore(a, path);
  ASSERT_TRUE(saved.ok) << saved.error;

  mvslam::map::MapStore b;
  b.setPlaceRecognizer(std::make_unique<mvslam::map::HbstPlaceRecognizer>());
  const auto loaded = mvslam::map::loadMapStore(b, path);
  ASSERT_TRUE(loaded.ok) << loaded.error;
  EXPECT_EQ(loaded.value->maps, 2);
  EXPECT_EQ(loaded.value->keyframes, 4);
  EXPECT_EQ(loaded.value->map_points, 5);
  EXPECT_EQ(b.stored().size(), 2u);
  EXPECT_NE(b.active()->id(), a.stored()[0]->id());   // fresh active map never collides with loaded ids
  EXPECT_NE(b.active()->id(), a.active()->id());
  EXPECT_GE(b.keyframeIds().peek(), a.keyframeIds().peek());
  EXPECT_GE(b.trackIds().peek(), a.trackIds().peek());
  EXPECT_EQ(static_cast<mvslam::map::HbstPlaceRecognizer*>(b.placeRecognizer())->numKeyframes(), 3u);  // descriptors re-registered

  auto first = b.find(a.stored()[0]->id());
  ASSERT_NE(first, nullptr);
  first->withReadLock([&](const mvslam::map::Map& mm) {
    EXPECT_EQ(mm.numKeyframes(), 3u);
    EXPECT_EQ(mm.numMapPoints(), 5u);
    auto kf = mm.keyframe(kfs[1]);
    EXPECT_NE(kf, nullptr);
    if (!kf) return 0;
    EXPECT_EQ(kf->timestamp(), 1001);
    EXPECT_LT((kf->T_wb().translation() - Vec3(0.1, 0.2, -0.3)).norm(), 1e-12);
    EXPECT_NEAR(kf->velocity().y(), 2.0, 1e-12);
    EXPECT_NEAR(kf->bias()(5), 6e-3, 1e-15);
    EXPECT_EQ(kf->keypoints().size(), 5u);
    if (kf->keypoints().size() != 5u) return 0;
    EXPECT_EQ(kf->keypoints()[2].track.value, 102u);
    EXPECT_TRUE(kf->keypoints()[2].has_right);
    EXPECT_TRUE(kf->keypoints()[4].outlier);
    EXPECT_NEAR(kf->keypoints()[3].depth, 4.5, 1e-12);
    EXPECT_EQ(kf->descriptors().size(), 160u);
    EXPECT_EQ(kf->mapPointSlots()[1].value, 101u);
    EXPECT_EQ(kf->prevKf(), kfs[0]);
    EXPECT_EQ(kf->nextKf(), kfs[2]);
    EXPECT_EQ(kf->parent(), kfs[0]);
    EXPECT_EQ(mm.keyframe(kfs[2])->loopEdges().count(kfs[0]), 1u);
    auto mp = mm.mapPoint(mps[3]);
    EXPECT_NE(mp, nullptr);
    if (!mp) return 0;
    EXPECT_EQ(mp->numObservations(), 3);
    EXPECT_EQ(mp->observations().at(kfs[2]).keypoint_index, 3);
    EXPECT_EQ(mp->anchor(), kfs[0]);
    EXPECT_EQ(mp->visibleCount(), 13);
    EXPECT_EQ(mp->descriptor()[0], 3);
    EXPECT_EQ(mm.covisibility().weight(kfs[0], kfs[2]), 5);
    EXPECT_EQ(mm.resolveTrack(TrackId{555}).value, 101u);
    const auto culled = mm.resolvePose(KeyframeId{999});
    EXPECT_TRUE(culled.has_value());
    const SE3 expected = SE3(SO3::exp(Vec3(0.01, 0, 0.02)), Vec3(0.1, 0.2, -0.3)) * SE3(SO3(), Vec3(0.5, 0, 0));  // T_w,kfs[1] * T_rc
    if (culled) EXPECT_LT((culled->translation() - expected.translation()).norm(), 1e-12);
    EXPECT_TRUE(mm.imuState().initialized);
    EXPECT_NEAR(mm.imuState().scale, 1.5, 1e-12);
    EXPECT_EQ(mm.lastKeyframe(), kfs[2]);
    EXPECT_EQ(mm.origins().size(), 1u);
    return 0;
  });

  const auto saved2 = mvslam::map::saveMapStore(b, path2);
  ASSERT_TRUE(saved2.ok) << saved2.error;
  const auto bytes1 = readFile(path), bytes2 = readFile(path2);
  EXPECT_GT(bytes1.size(), 1000u);
  EXPECT_EQ(bytes1, bytes2);
  std::remove(path.c_str());
  std::remove(path2.c_str());
}

TEST(MapArchive, RejectsForeignFiles) {
  const std::string path = "test_map_archive_bogus.bin";
  { std::ofstream f(path, std::ios::binary); f << "definitely not a map"; }
  mvslam::map::MapStore s;
  const auto r = mvslam::map::loadMapStore(s, path);
  EXPECT_FALSE(r.ok);
  std::remove(path.c_str());
  EXPECT_FALSE(mvslam::map::loadMapStore(s, "/nonexistent/dir/map.mvs").ok);
}
