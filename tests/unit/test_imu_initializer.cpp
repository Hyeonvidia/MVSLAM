#include <gtest/gtest.h>

#include <random>

#include "backend/imu/imu_initializer.hpp"
#include "backend/imu/inertial_optimization.hpp"
#include "backend/solvers/solver.hpp"
#include "imu_simulation.hpp"
#include "map/map_store.hpp"

using namespace mvslam::core;
using namespace mvslam::backend::imu;
using mvslam::testing::ImuTrajectory;

namespace {
mvslam::sensors::ImuCalibration calib() {
  mvslam::sensors::ImuCalibration c;
  c.rate_hz = 200; c.noise_gyro = 1.7e-4; c.noise_acc = 2e-3; c.walk_gyro = 1.9e-5; c.walk_acc = 3e-3; c.gravity = 9.81;
  return c;
}
std::shared_ptr<const Preintegration> preintegrate(const ImuTrajectory& traj, double t0, double t1, const Vec6& bias_lin) {
  Preintegrator pi(calib());
  pi.reset(bias_lin);
  const int n = static_cast<int>(std::round((t1 - t0) * 200));
  const double dt = (t1 - t0) / n;
  for (int i = 0; i < n; ++i) {
    const double ta = t0 + i * dt, tb = ta + dt;
    pi.integrate(0.5 * (traj.acc(ta) + traj.acc(tb)), 0.5 * (traj.gyro(ta) + traj.gyro(tb)), dt);
  }
  return pi.snapshot();
}
// Keyframe chain every `step` seconds in a map frame rotated by R_mw and scaled by 1/s (mono-like).
std::vector<KeyframeInertialState> chain(const ImuTrajectory& traj, double t_end, double step, const SO3& R_mw, double s) {
  std::vector<KeyframeInertialState> c;
  for (double t = 0; t <= t_end + 1e-9; t += step) {
    KeyframeInertialState k;
    k.id = KeyframeId{static_cast<std::uint64_t>(c.size() + 1)};
    k.timestamp = static_cast<std::int64_t>(std::llround(t * 1e9));
    k.T_wb = SE3(R_mw * traj.R(t), (R_mw * traj.p(t)) / s);
    if (!c.empty()) k.preint = preintegrate(traj, t - step, t, Vec6::Zero());
    c.push_back(k);
  }
  return c;
}
}  // namespace

TEST(InertialOnlyOptimization, RecoversScaleGravityAndBiases) {
  ImuTrajectory traj;
  traj.bias << 0.01, -0.02, 0.015, 0.05, -0.03, 0.02;
  const double s = 3.0;
  const SO3 R_mw = SO3::exp(Vec3(0.5, -0.4, 0.3));
  auto c = chain(traj, 3.0, 0.25, R_mw, s);
  ASSERT_EQ(c.size(), 13u);
  const auto v0 = initialVelocities(c);
  for (std::size_t i = 0; i < c.size(); ++i) c[i].v_w = v0[i];
  const SO3 R_wg0 = initialGravityDirection(c);
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));
  // Stage 1 (ORB-SLAM3 mono priors): gravity/scale/velocities, accelerometer bias pinned near zero.
  InertialOnlyOptions o1;
  o1.prior_gyro = 1e2; o1.prior_acc = 1e10;
  auto r1 = inertialOnlyOptimization(c, R_wg0, 1.0, Vec6::Zero(), o1, *solver);
  ASSERT_TRUE(r1.ok);
  EXPECT_NEAR(r1.scale, s, 0.03 * s);
  const Vec3 g_map_true = R_mw * traj.g_w;  // gravity expressed in the map frame
  const Vec3 g_est = r1.R_wg * Vec3(0, 0, -9.81);
  EXPECT_LT(std::acos(g_est.normalized().dot(g_map_true.normalized())) * 180.0 / M_PI, 1.0);
  EXPECT_LT(r1.final_chi2, r1.initial_chi2);
  // Stage 2 (no priors) starting from stage 1: biases and scale converge.
  for (std::size_t i = 0; i < c.size(); ++i) c[i].v_w = r1.velocities[i];
  InertialOnlyOptions o2;
  o2.prior_gyro = 0; o2.prior_acc = 0;
  auto r2 = inertialOnlyOptimization(c, r1.R_wg, r1.scale, r1.bias, o2, *solver);
  ASSERT_TRUE(r2.ok);
  EXPECT_NEAR(r2.scale, s, 0.01 * s);
  EXPECT_LT((r2.bias.head<3>() - traj.bias.head<3>()).norm(), 2e-3);
  EXPECT_LT((r2.bias.tail<3>() - traj.bias.tail<3>()).norm(), 2e-2);
  EXPECT_GT(r2.scale_variance, 0);
  // Velocities in the map frame (scaled): v_map = R_mw v / s.
  for (std::size_t i = 0; i + 1 < c.size(); ++i) {
    const double t = 0.25 * static_cast<double>(i);
    EXPECT_LT((r2.velocities[i] - (R_mw * traj.v(t)) / s).norm(), 0.05) << i;
  }
}

TEST(ImuInitializer, InitializesASyntheticMonoMap) {
  ImuTrajectory traj;
  traj.bias << 0.005, -0.01, 0.008, 0.03, -0.02, 0.01;
  const double s = 2.0;
  const SO3 R_mw = SO3::exp(Vec3(-0.3, 0.2, 0.4));
  const auto c = chain(traj, 2.5, 0.25, R_mw, s);  // 11 keyframes over 2.5 s (mono needs >= 10 and 2 s)
  mvslam::RigConfig rc;
  mvslam::CameraConfig cc; cc.width = 752; cc.height = 480; cc.intrinsics = {458.654, 457.296, 367.215, 248.375};
  rc.cameras.push_back(cc);
  mvslam::ImuConfig ic; rc.imu = ic;
  mvslam::sensors::CameraRig rig(rc);
  mvslam::Config cfg;
  cfg.sensor = mvslam::SensorMode::Mono;
  cfg.inertial = mvslam::InertialMode::Imu;
  cfg.execution = mvslam::ExecutionMode::Offline;
  cfg.backend.lba_min_covisibility = 1;
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));

  // Map with the chain as keyframes and 60 points observed by all of them (exact projections in the map frame).
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1, 1);
  mvslam::map::MapStore store;
  auto map = store.active();
  std::vector<Vec3> X_map;
  for (int i = 0; i < 60; ++i) X_map.push_back((R_mw * (traj.p(1.0) + Vec3(2 * u(rng), 1.5 * u(rng), 4 + 1.5 * u(rng)))) / s);
  map->withWriteLock([&](mvslam::map::Map& m) {
    for (std::size_t i = 0; i < c.size(); ++i) {
      auto kf = std::make_shared<mvslam::map::KeyFrame>(c[i].id, c[i].timestamp, c[i].T_wb);
      kf->setPreintegration(c[i].preint);
      if (i > 0) { kf->setPrevKf(c[i - 1].id); m.keyframe(c[i - 1].id)->setNextKf(c[i].id); }
      for (std::size_t p = 0; p < X_map.size(); ++p) {
        mvslam::map::Keypoint kp;
        kp.uv = kp.uv_raw = rig.camera(0).project(rig.T_cb(0) * (c[i].T_wb.inverse() * X_map[p]));
        kp.track = TrackId{p + 1};
        kf->keypoints().push_back(kp);
      }
      kf->mapPointSlots().assign(X_map.size(), MapPointId{});
      m.addKeyFrame(kf);
    }
    for (std::size_t p = 0; p < X_map.size(); ++p) {
      const MapPointId id{p + 1};
      m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(id, X_map[p], c.front().id));
      for (const auto& k : c) m.addObservation(id, k.id, {static_cast<int>(p), 0});
    }
    for (const auto& k : c) m.updateCovisibility(k.id, 1);
    return 0;
  });

  ImuInitializer init(cfg, rig, solver);
  const auto step = init.step(*map, c.back().id);
  EXPECT_EQ(step.before, ImuInitState::Uninitialized);
  EXPECT_EQ(step.after, ImuInitState::Initialized);
  EXPECT_TRUE(step.applied_scaled_rotation);
  EXPECT_NEAR(step.scale, s, 0.05 * s);
  map->withReadLock([&](const mvslam::map::Map& m) {
    EXPECT_TRUE(m.imuState().initialized);
    // After alignment: metric scale (inter-keyframe distances) and gravity along -z (height differences match).
    const auto kfs = m.keyframes();
    for (std::size_t i = 1; i < kfs.size(); ++i) {
      const double t_i = 0.25 * static_cast<double>(i);
      const double d_true = (traj.p(t_i) - traj.p(0)).norm();
      const double d_map = (kfs[i]->T_wb().translation() - kfs[0]->T_wb().translation()).norm();
      EXPECT_NEAR(d_map, d_true, 0.05 * d_true + 0.02) << i;
      const double dz_true = traj.p(t_i).z() - traj.p(0).z();
      const double dz_map = kfs[i]->T_wb().translation().z() - kfs[0]->T_wb().translation().z();
      EXPECT_NEAR(dz_map, dz_true, 0.05 * std::abs(dz_true) + 0.03) << i;
      EXPECT_LT((kfs[i]->bias().head<3>() - traj.bias.head<3>()).norm(), 5e-3) << i;
    }
    return 0;
  });
}
