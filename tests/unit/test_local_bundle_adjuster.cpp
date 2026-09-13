#include <gtest/gtest.h>

#include <random>

#include "backend/local_ba/local_bundle_adjuster.hpp"
#include "backend/solvers/solver.hpp"
#include "map/map_store.hpp"

using namespace mvslam::core;

TEST(LocalBundleAdjuster, ConvergesToGroundTruthOnStereoWindow) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 752; c0.height = 480; c0.intrinsics = {458.654, 457.296, 367.215, 248.375};
  mvslam::CameraConfig c1 = c0; c1.T_bc = {1, 0, 0, 0.11, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  rc.cameras = {c0, c1};
  mvslam::sensors::CameraRig rig(rc);
  mvslam::Config cfg;
  cfg.sensor = mvslam::SensorMode::Stereo;
  cfg.execution = mvslam::ExecutionMode::Offline;
  cfg.backend.lba_iters = 10;
  cfg.backend.lba_min_covisibility = 15;
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));
  mvslam::backend::LocalBundleAdjuster lba(cfg, rig, solver);

  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-1, 1);
  const int n_kf = 6, n_pt = 80;
  std::vector<Vec3> X;
  for (int i = 0; i < n_pt; ++i) X.emplace_back(2.0 * u(rng), 1.2 * u(rng), 4.0 + 1.5 * u(rng));
  std::vector<SE3> gt;
  for (int k = 0; k < n_kf; ++k) gt.emplace_back(SO3::exp(Vec3(0, 0.02 * k, 0)), Vec3(0.12 * k, 0.01 * k, 0.0));

  mvslam::map::MapStore store;
  auto map = store.active();
  std::vector<KeyframeId> kfs;
  std::vector<MapPointId> mps;
  map->withWriteLock([&](mvslam::map::Map& m) {
    for (int k = 0; k < n_kf; ++k) {
      const auto id = store.keyframeIds().next();
      SE3 T = gt[static_cast<std::size_t>(k)];
      if (k > 0) T = T * SE3::exp((Vec6() << 0.02 * u(rng), 0.02 * u(rng), 0.02 * u(rng), 0.01 * u(rng), 0.01 * u(rng), 0.01 * u(rng)).finished());
      auto kf = std::make_shared<mvslam::map::KeyFrame>(id, k, T);
      for (int i = 0; i < n_pt; ++i) {
        mvslam::map::Keypoint kp;
        const Vec3 pc0 = rig.T_cb(0) * (gt[static_cast<std::size_t>(k)].inverse() * X[static_cast<std::size_t>(i)]);
        const Vec3 pc1 = rig.T_cb(1) * (gt[static_cast<std::size_t>(k)].inverse() * X[static_cast<std::size_t>(i)]);
        kp.uv = kp.uv_raw = rig.camera(0).project(pc0);
        kp.has_right = true;
        kp.uv_right = rig.camera(1).project(pc1);
        kp.track = TrackId{static_cast<std::uint64_t>(i + 1)};
        kf->keypoints().push_back(kp);
      }
      kf->mapPointSlots().assign(static_cast<std::size_t>(n_pt), MapPointId{});
      if (!kfs.empty()) kf->setPrevKf(kfs.back());
      m.addKeyFrame(kf);
      kfs.push_back(id);
    }
    for (int i = 0; i < n_pt; ++i) {
      const MapPointId id{static_cast<std::uint64_t>(i + 1)};
      const Vec3 noisy = X[static_cast<std::size_t>(i)] + Vec3(0.05 * u(rng), 0.05 * u(rng), 0.08 * u(rng));
      m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, noisy, kfs.front()));
      for (int k = 0; k < n_kf; ++k) m.addObservation(id, kfs[static_cast<std::size_t>(k)], {i, 0});
      mps.push_back(id);
    }
    for (auto k : kfs) m.updateCovisibility(k, cfg.backend.lba_min_covisibility);
    return 0;
  });

  const auto rep = lba.run(*map, kfs.back(), std::stop_token{}, nullptr);
  EXPECT_TRUE(rep.ran);
  EXPECT_TRUE(rep.applied);
  EXPECT_FALSE(rep.preempted);
  EXPECT_EQ(rep.keyframes, n_kf);
  EXPECT_EQ(rep.points, n_pt);
  EXPECT_EQ(rep.observations, 2 * n_kf * n_pt);  // left + right
  EXPECT_EQ(rep.outliers, 0);
  EXPECT_LT(rep.final_chi2, 1e-3);
  EXPECT_LT(rep.final_chi2, rep.initial_chi2);

  map->withReadLock([&](const mvslam::map::Map& m) {
    for (int k = 0; k < n_kf; ++k) {
      const Vec6 err = (gt[static_cast<std::size_t>(k)].inverse() * m.keyframe(kfs[static_cast<std::size_t>(k)])->T_wb()).log();
      EXPECT_LT(err.head<3>().norm(), 1e-4) << "kf " << k;   // translation [m]
      EXPECT_LT(err.tail<3>().norm(), 1e-4) << "kf " << k;   // rotation [rad]
    }
    for (int i = 0; i < n_pt; ++i) EXPECT_LT((m.mapPoint(mps[static_cast<std::size_t>(i)])->position() - X[static_cast<std::size_t>(i)]).norm(), 1e-4) << i;
    return 0;
  });
}

TEST(LocalBundleAdjuster, SkipsTinyWindows) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 752; c0.height = 480; c0.intrinsics = {458.654, 457.296, 367.215, 248.375};
  rc.cameras = {c0};
  mvslam::sensors::CameraRig rig(rc);
  mvslam::Config cfg;
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));
  mvslam::backend::LocalBundleAdjuster lba(cfg, rig, solver);
  mvslam::map::MapStore store;
  auto map = store.active();
  const auto k = store.keyframeIds().next();
  map->withWriteLock([&](mvslam::map::Map& m) { m.addKeyFrame(std::make_shared<mvslam::map::KeyFrame>(k, 0, SE3())); return 0; });
  const auto rep = lba.run(*map, k, std::stop_token{}, nullptr);
  EXPECT_FALSE(rep.ran);
  EXPECT_FALSE(rep.applied);
}
