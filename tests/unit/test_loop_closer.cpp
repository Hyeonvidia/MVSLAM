// Loop detection, same-map correction and map merge on a synthetic scene: a wall of described 3-D points
// seen by an early group of keyframes (trusted) and, after a chain of keyframes elsewhere, by a late group
// whose poses and points carry an injected drift D. The detector must recover D^-1 from ORB-style descriptor
// matches + P3P + votes; the closer must pull the late group back and fuse the duplicated points.
#include <gtest/gtest.h>

#include <array>
#include <random>

#include "backend/loop/loop_closer.hpp"
#include "core/log.hpp"
#include "backend/pose_optimizer.hpp"
#include "frontend/opencv/pnp_solver.hpp"
#include "map/hbst_place_recognizer.hpp"
#include "map/map_store.hpp"
#include "sensors/camera_rig.hpp"

using namespace mvslam::core;
namespace map = mvslam::map;
namespace backend = mvslam::backend;
namespace frontend = mvslam::frontend;
namespace sensors = mvslam::sensors;

namespace {
constexpr int kStride = 32;

struct Scene {
  mvslam::Config cfg;
  std::unique_ptr<sensors::CameraRig> rig;
  std::vector<Vec3> points;                          // true positions
  std::vector<std::array<std::uint8_t, 32>> desc;    // one descriptor per physical point
  std::mt19937 rng{11};
};

Scene makeScene() {
  Scene s;
  mvslam::CameraConfig c;
  c.width = 640; c.height = 480; c.intrinsics = {400, 400, 320, 240};
  s.cfg.sensor = mvslam::SensorMode::Stereo;   // fixed scale
  s.cfg.rig.cameras = {c};
  s.cfg.loop.min_keyframe_gap = 10;
  s.cfg.loop.vote_keyframes = 4;
  s.cfg.frontend.reloc_ransac_iters = 500;
  s.rig = std::make_unique<sensors::CameraRig>(s.cfg.rig);
  std::uniform_real_distribution<double> ux(-4, 4), uy(-3, 3), uz(5, 12);
  std::uniform_int_distribution<int> bit(0, 255);
  for (int i = 0; i < 700; ++i) {
    s.points.emplace_back(ux(s.rng), uy(s.rng), uz(s.rng));
    std::array<std::uint8_t, 32> d{};
    for (auto& b : d) b = static_cast<std::uint8_t>(bit(s.rng));
    s.desc.push_back(d);
  }
  return s;
}

// Keyframe at true pose T_true, stored in the map with the frame transform F (estimate = F * T_true, points = F * X).
// `ids`: map point id per scene point (invalid = not yet in this map; created when `create`).
KeyframeId addKeyframe(Scene& s, map::MapStore& store, map::Map& m, const SE3& T_true, const SE3& F, std::vector<MapPointId>& ids, bool create,
                       KeyframeId parent) {
  const auto& cam = s.rig->camera(0);
  const auto id = store.keyframeIds().next();
  auto kf = std::make_shared<map::KeyFrame>(id, static_cast<std::int64_t>(id.value) * 100000000, F * T_true);
  std::normal_distribution<double> px(0, 0.3);
  std::uniform_int_distribution<int> flipbit(0, 255);
  std::vector<std::pair<int, MapPointId>> obs;
  const SE3 T_cw = T_true.inverse();
  for (std::size_t i = 0; i < s.points.size(); ++i) {
    const Vec3 pc = T_cw * s.points[i];
    if (pc.z() < 0.5) continue;
    const Vec2 uv = cam.project(pc) + Vec2(px(s.rng), px(s.rng));
    if (!cam.inImage(uv, 8)) continue;
    map::Keypoint kp;
    kp.uv = kp.uv_raw = uv;
    std::array<std::uint8_t, 32> d = s.desc[i];
    for (int f = 0; f < 4; ++f) { const int b = flipbit(s.rng); d[static_cast<std::size_t>(b / 8)] ^= static_cast<std::uint8_t>(1 << (b % 8)); }
    kf->descriptors().insert(kf->descriptors().end(), d.begin(), d.end());
    if (!ids[i].valid() && create) {
      ids[i] = store.mapPointIds().next();
      auto mp = std::make_shared<map::MapPoint>(ids[i], F * s.points[i], id);
      mp->setDescriptor(std::vector<std::uint8_t>(s.desc[i].begin(), s.desc[i].end()));
      m.addMapPoint(mp);
    }
    kp.track = TrackId{ids[i].valid() ? ids[i].value : store.trackIds().next().value};
    obs.emplace_back(static_cast<int>(kf->keypoints().size()), ids[i]);
    kf->keypoints().push_back(kp);
  }
  kf->mapPointSlots().assign(kf->keypoints().size(), MapPointId{});
  if (parent.valid()) { kf->setParent(parent); kf->setPrevKf(parent); if (auto p = m.keyframe(parent)) { p->children().insert(id); p->setNextKf(id); } }
  m.addKeyFrame(kf);
  for (const auto& [slot, mpid] : obs) if (mpid.valid()) m.addObservation(mpid, id, {slot, 0});
  m.updateCovisibility(id, 15);
  m.setLastKeyframe(id);
  if (auto* pr = store.placeRecognizer(); pr && !kf->descriptors().empty()) pr->add(id, m.id(), kf->descriptors(), kStride);
  return id;
}

SE3 groupPose(int i, double y) { return SE3(SO3::exp(Vec3(0, 0.02 * (i - 2), 0)), Vec3(0.25 * (i - 2), y, 0)); }

struct Harness {
  Scene scene = makeScene();
  map::MapStore store;
  std::shared_ptr<backend::graph::ISolver> solver = std::shared_ptr<backend::graph::ISolver>(backend::graph::makeSolver(mvslam::SolverBackend::Ceres));
  std::unique_ptr<frontend::cv_impl::P3pRansacSolver> p3p;
  std::unique_ptr<backend::MotionOnlyBa> ba;
  Harness() {
    mvslam::core::initLogging("debug", true, "");
    store.setPlaceRecognizer(std::make_unique<map::HbstPlaceRecognizer>());
    mvslam::FrontendConfig fc = scene.cfg.frontend;
    fc.ransac_iters = fc.reloc_ransac_iters;
    p3p = std::make_unique<frontend::cv_impl::P3pRansacSolver>(fc, *scene.rig);
    ba = std::make_unique<backend::MotionOnlyBa>(*scene.rig, solver, fc, scene.cfg.backend);
  }
};
}  // namespace

TEST(LoopClosing, DetectsAndCorrectsSameMapLoop) {
  Harness h;
  auto map = h.store.active();
  const SE3 D(SO3::exp(Vec3(0.01, -0.02, 0.05)), Vec3(0.5, -0.3, 0.2));   // drift carried by the late group
  std::vector<MapPointId> early_ids(h.scene.points.size()), late_ids(h.scene.points.size());
  std::vector<KeyframeId> early, chain, late;
  std::vector<SE3> late_true;
  map->withWriteLock([&](map::Map& m) {
    KeyframeId prev;
    for (int i = 0; i < 5; ++i) { prev = addKeyframe(h.scene, h.store, m, groupPose(i, 0.0), SE3(), early_ids, true, prev); early.push_back(prev); }
    std::vector<MapPointId> none(h.scene.points.size());
    for (int i = 0; i < 24; ++i) {  // keyframes looking away from the wall: chain only (no observations)
      const SE3 T(SO3::exp(Vec3(0, M_PI, 0)), Vec3(1.0 + 0.3 * i, 0, -2));
      prev = addKeyframe(h.scene, h.store, m, T, SE3(), none, false, prev);
      chain.push_back(prev);
    }
    for (int i = 0; i < 5; ++i) {
      late_true.push_back(groupPose(i, 0.4));
      prev = addKeyframe(h.scene, h.store, m, late_true.back(), D, late_ids, true, prev);
      late.push_back(prev);
    }
    return 0;
  });
  const auto points_before = map->withReadLock([](const map::Map& m) { return m.numMapPoints(); });
  backend::loop::LoopDetector det(h.scene.cfg, *h.scene.rig, h.store, *h.p3p, *h.ba);
  const auto hyp = det.detect(late.back());
  ASSERT_TRUE(hyp.has_value());
  EXPECT_EQ(hyp->candidate_map, map->id());
  EXPECT_TRUE(std::find(early.begin(), early.end(), hyp->candidate) != early.end());
  EXPECT_GE(hyp->inliers, 30);
  EXPECT_GE(hyp->votes, 3);
  EXPECT_NEAR(hyp->S_cq.scale(), 1.0, 1e-9);
  // S_cq maps the drifted query pose back onto its true pose.
  const SE3 T_q = D * late_true.back();
  const SE3 corrected(hyp->S_cq.rxso3().so3() * T_q.so3(), hyp->S_cq * T_q.translation());
  EXPECT_LT((corrected.translation() - late_true.back().translation()).norm(), 0.03);
  EXPECT_LT((corrected.so3().inverse() * late_true.back().so3()).log().norm(), 0.01);

  backend::loop::LoopCloser closer(h.scene.cfg, *h.scene.rig, h.store, h.solver, nullptr);
  const auto rep = closer.close(*hyp, std::stop_token{});
  ASSERT_TRUE(rep.closed);
  EXPECT_FALSE(rep.merged);
  EXPECT_GT(rep.fused, 100);           // duplicated late points fused into the early ones
  EXPECT_GE(rep.loop_edges, 1);
  EXPECT_TRUE(rep.pgo.applied);
  map->withReadLock([&](const map::Map& m) {
    EXPECT_LT(m.numMapPoints(), points_before - 100);
    for (std::size_t i = 0; i < late.size(); ++i) {
      const auto kf = m.keyframe(late[i]);
      EXPECT_TRUE(kf);
      if (!kf) continue;
      EXPECT_LT((kf->T_wb().translation() - late_true[i].translation()).norm(), 0.05) << "late keyframe " << i;
    }
    for (auto k : early) EXPECT_LT((m.keyframe(k)->T_wb().translation() - groupPose(static_cast<int>(k.value) - 1, 0.0).translation()).norm(), 0.05);
    EXPECT_TRUE(m.keyframe(late.back())->loopEdges().count(hyp->candidate));
    EXPECT_GT(m.covisibility().weight(late.back(), hyp->candidate), 0);   // fused points connect the two groups
    return 0;
  });
  det.closed(late.back());
  EXPECT_FALSE(det.detect(late.back()).has_value());   // cooldown
}

TEST(LoopClosing, MergesActiveMapIntoStoredMap) {
  Harness h;
  auto B = h.store.active();   // trusted map, becomes stored
  const SE3 D(SO3::exp(Vec3(0, 0, -0.08)), Vec3(-0.6, 0.4, 0.1));
  std::vector<MapPointId> b_ids(h.scene.points.size()), a_ids(h.scene.points.size());
  std::vector<KeyframeId> b_kfs, a_kfs;
  std::vector<SE3> a_true;
  B->withWriteLock([&](map::Map& m) { KeyframeId prev; for (int i = 0; i < 5; ++i) { prev = addKeyframe(h.scene, h.store, m, groupPose(i, 0.0), SE3(), b_ids, true, prev); b_kfs.push_back(prev); } return 0; });
  auto A = h.store.startNewMap();
  A->withWriteLock([&](map::Map& m) {
    KeyframeId prev;
    for (int i = 0; i < 5; ++i) { a_true.push_back(groupPose(i, -0.3)); prev = addKeyframe(h.scene, h.store, m, a_true.back(), D, a_ids, true, prev); a_kfs.push_back(prev); }
    return 0;
  });
  ASSERT_EQ(h.store.all().size(), 2u);
  backend::loop::LoopDetector det(h.scene.cfg, *h.scene.rig, h.store, *h.p3p, *h.ba);
  const auto hyp = det.detect(a_kfs.back());
  ASSERT_TRUE(hyp.has_value());
  EXPECT_EQ(hyp->query_map, A->id());
  EXPECT_EQ(hyp->candidate_map, B->id());
  int events_merged = 0, events_corrected = 0;
  auto sub = h.store.events().subscribe([&](const mvslam::MapEvent& e) {
    if (std::get_if<mvslam::MapMergedEvent>(&e)) ++events_merged;
    if (const auto* pc = std::get_if<mvslam::PosesCorrectedEvent>(&e)) events_corrected += static_cast<int>(pc->corrections.size());
  });
  backend::loop::LoopCloser closer(h.scene.cfg, *h.scene.rig, h.store, h.solver, nullptr);
  const auto rep = closer.close(*hyp, std::stop_token{});
  ASSERT_TRUE(rep.merged);
  EXPECT_GT(rep.fused, 100);
  EXPECT_EQ(events_merged, 1);
  EXPECT_EQ(events_corrected, 5);   // the five keyframes of A moved
  EXPECT_EQ(h.store.active()->id(), B->id());
  EXPECT_EQ(h.store.all().size(), 1u);
  EXPECT_EQ(h.store.resolveMap(A->id()), B->id());
  B->withReadLock([&](const map::Map& m) {
    EXPECT_EQ(m.numKeyframes(), 10u);
    EXPECT_EQ(m.lastKeyframe(), a_kfs.back());
    for (std::size_t i = 0; i < a_kfs.size(); ++i) {
      const auto kf = m.keyframe(a_kfs[i]);
      EXPECT_TRUE(kf);
      if (!kf) continue;
      EXPECT_LT((kf->T_wb().translation() - a_true[i].translation()).norm(), 0.05) << "merged keyframe " << i;
    }
    EXPECT_GT(m.covisibility().weight(a_kfs.back(), hyp->candidate), 0);
    return 0;
  });
  A->withReadLock([](const map::Map& m) { EXPECT_EQ(m.numKeyframes(), 0u); return 0; });
}
