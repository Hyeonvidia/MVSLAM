#include <gtest/gtest.h>

#include <random>

#include "backend/local_ba/inertial_bundle_adjuster.hpp"
#include "backend/solvers/solver.hpp"
#include "imu_simulation.hpp"
#include "map/map_store.hpp"

using namespace mvslam::core;
using mvslam::testing::ImuTrajectory;

namespace {
mvslam::sensors::ImuCalibration calib() {
  mvslam::sensors::ImuCalibration c;
  c.rate_hz = 200; c.noise_gyro = 1.7e-4; c.noise_acc = 2e-3; c.walk_gyro = 1.9e-5; c.walk_acc = 3e-3; c.gravity = 9.81;
  return c;
}
std::shared_ptr<const mvslam::sensors::imu::Preintegration> preintegrate(const ImuTrajectory& traj, double t0, double t1, const Vec6& bias_lin) {
  mvslam::sensors::imu::Preintegrator pi(calib());
  pi.reset(bias_lin);
  const int n = static_cast<int>(std::round((t1 - t0) * 200));
  const double dt = (t1 - t0) / n;
  for (int i = 0; i < n; ++i) {
    const double ta = t0 + i * dt, tb = ta + dt;
    pi.integrate(0.5 * (traj.acc(ta) + traj.acc(tb)), 0.5 * (traj.gyro(ta) + traj.gyro(tb)), dt);
  }
  return pi.snapshot();
}
}  // namespace

// Stereo rig, exact observations, exact IMU (gravity along -z, true biases), perturbed keyframe states:
// the visual-inertial BA must return to the truth and leave the IMU residuals near zero.
TEST(InertialBundleAdjuster, RecoversPerturbedWindowWithExactData) {
  ImuTrajectory traj;
  traj.bias << 0.01, -0.02, 0.015, 0.05, -0.03, 0.02;
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 752; c0.height = 480; c0.intrinsics = {458.654, 457.296, 367.215, 248.375};
  mvslam::CameraConfig c1 = c0; c1.T_bc = {1, 0, 0, 0.11, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  rc.cameras = {c0, c1};
  mvslam::ImuConfig ic; ic.gravity = 9.81; rc.imu = ic;
  mvslam::sensors::CameraRig rig(rc);
  mvslam::Config cfg;
  cfg.sensor = mvslam::SensorMode::Stereo;
  cfg.inertial = mvslam::InertialMode::Imu;
  cfg.execution = mvslam::ExecutionMode::Offline;
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));

  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1, 1);
  const int n_kf = 8;
  const double step = 0.25;
  std::vector<Vec3> X;
  for (int i = 0; i < 80; ++i) X.push_back(traj.p(1.0) + traj.R(1.0) * Vec3(2 * u(rng), 1.5 * u(rng), 4 + 1.5 * u(rng)));
  mvslam::map::MapStore store;
  auto map = store.active();
  std::vector<KeyframeId> ids;
  map->withWriteLock([&](mvslam::map::Map& m) {
    for (int k = 0; k < n_kf; ++k) {
      const double t = step * k;
      const auto id = store.keyframeIds().next();
      SE3 T = traj.T(t);
      Vec3 v = traj.v(t);
      Vec6 b = traj.bias;
      if (k > 0) {  // perturb everything but the boundary keyframe
        T = T * SE3::exp((Vec6() << 0.03 * u(rng), 0.03 * u(rng), 0.03 * u(rng), 0.01 * u(rng), 0.01 * u(rng), 0.01 * u(rng)).finished());
        v += Vec3(0.1 * u(rng), 0.1 * u(rng), 0.1 * u(rng));
        b += (Vec6() << 0.002 * u(rng), 0.002 * u(rng), 0.002 * u(rng), 0.02 * u(rng), 0.02 * u(rng), 0.02 * u(rng)).finished();
      }
      auto kf = std::make_shared<mvslam::map::KeyFrame>(id, static_cast<std::int64_t>(std::llround(t * 1e9)), T);
      kf->setVelocity(v);
      kf->setBias(b);
      if (k > 0) { kf->setPreintegration(preintegrate(traj, t - step, t, traj.bias)); kf->setPrevKf(ids.back()); m.keyframe(ids.back())->setNextKf(id); }
      for (std::size_t p = 0; p < X.size(); ++p) {
        mvslam::map::Keypoint kp;
        const Vec3 pc0 = rig.T_cb(0) * (traj.T(t).inverse() * X[p]);
        kp.uv = kp.uv_raw = rig.camera(0).project(pc0);
        kp.has_right = true;
        kp.uv_right = rig.camera(1).project(rig.T_cb(1) * (traj.T(t).inverse() * X[p]));
        kp.track = TrackId{p + 1};
        kf->keypoints().push_back(kp);
      }
      kf->mapPointSlots().assign(X.size(), MapPointId{});
      m.addKeyFrame(kf);
      ids.push_back(id);
    }
    for (std::size_t p = 0; p < X.size(); ++p) {
      const MapPointId id{p + 1};
      m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, X[p] + Vec3(0.03 * u(rng), 0.03 * u(rng), 0.05 * u(rng)), ids.front()));
      for (auto k : ids) m.addObservation(id, k, {static_cast<int>(p), 0});
    }
    for (auto k : ids) m.updateCovisibility(k, 1);
    return 0;
  });

  mvslam::backend::InertialBundleAdjuster ba(cfg, rig, solver);
  std::vector<KeyframeId> window(ids.begin() + 1, ids.end());  // ids[0] is the fixed boundary
  mvslam::backend::InertialBaOptions opt;
  opt.iterations = 15;
  const auto rep = ba.run(*map, window, opt);
  EXPECT_TRUE(rep.applied);
  EXPECT_EQ(rep.imu_factors, n_kf - 1);
  EXPECT_LT(rep.final_cost, rep.initial_cost);
  map->withReadLock([&](const mvslam::map::Map& m) {
    for (int k = 1; k < n_kf; ++k) {
      const double t = step * k;
      const auto kf = m.keyframe(ids[static_cast<std::size_t>(k)]);
      const Vec6 err = (traj.T(t).inverse() * kf->T_wb()).log();
      // The 200 Hz first-order IMU integral is not exact (~1e-3 m over 0.25 s), so the VI compromise stays within a few mm.
      EXPECT_LT(err.head<3>().norm(), 5e-3) << "kf " << k << " translation";
      EXPECT_LT(err.tail<3>().norm(), 2e-3) << "kf " << k << " rotation";
      EXPECT_LT((kf->velocity() - traj.v(t)).norm(), 2e-2) << "kf " << k << " velocity";
      EXPECT_LT((kf->bias().head<3>() - traj.bias.head<3>()).norm(), 3e-3) << "kf " << k << " gyro bias";
    }
    return 0;
  });
}
