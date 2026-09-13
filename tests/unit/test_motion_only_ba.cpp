#include <gtest/gtest.h>

#include <random>

#include "backend/pose_optimizer.hpp"
#include "backend/solvers/solver.hpp"

using namespace mvslam::core;

TEST(MotionOnlyBa, RecoversPoseAndRejectsOutliers) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig cc; cc.width = 752; cc.height = 480; cc.intrinsics = {458.654, 457.296, 367.215, 248.375};
  rc.cameras.push_back(cc);
  mvslam::sensors::CameraRig rig(rc);
  mvslam::FrontendConfig f; mvslam::BackendConfig b;
  auto solver = std::shared_ptr<mvslam::backend::graph::ISolver>(mvslam::backend::graph::makeSolver(mvslam::SolverBackend::Ceres));
  mvslam::backend::MotionOnlyBa ba(rig, solver, f, b);

  std::mt19937 rng(1);
  std::uniform_real_distribution<double> u(-1, 1);
  const SE3 gt(Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Vec3::UnitZ())), Vec3(0.2, -0.1, 0.05));
  mvslam::frontend::Frame fr;
  fr.T_wb = gt * SE3::exp((Vec6() << 0.1, -0.05, 0.08, 0.03, -0.02, 0.04).finished());  // perturbed init
  const auto& cam = rig.camera(0);
  for (int i = 0; i < 60; ++i) {
    mvslam::frontend::Track t;
    t.is3d = true;
    t.X_w = gt * Vec3(2 * u(rng), 1.5 * u(rng), 3 + 2 * u(rng));
    t.uv = cam.project(gt.inverse() * t.X_w) + Vec2(0.3 * u(rng), 0.3 * u(rng));
    if (i % 10 == 0) t.uv += Vec2(40, -30);  // 6 gross outliers
    fr.tracks.push_back(t);
  }
  const auto res = ba.optimize(fr, 0);
  EXPECT_TRUE(res.ok);
  EXPECT_GE(res.inliers, 50);
  EXPECT_EQ(ba.lastOutliers().size(), 6u);
  const Vec6 err = (gt.inverse() * fr.T_wb).log();
  EXPECT_LT(err.head<3>().norm(), 0.005);
  EXPECT_LT(err.tail<3>().norm(), 0.002);
}
