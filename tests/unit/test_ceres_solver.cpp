#include <gtest/gtest.h>

#include <random>

#include "backend/graph/factors/reprojection_factor.hpp"
#include "backend/solvers/solver.hpp"
#include "sensors/camera_model.hpp"

using namespace mvslam::core;
using namespace mvslam::backend::graph;

TEST(CeresSolver, SmallBundleAdjustmentConverges) {
  mvslam::CameraConfig cc; cc.width = 640; cc.height = 480; cc.intrinsics = {500, 500, 320, 240};
  mvslam::sensors::PinholeCamera cam(cc);
  std::mt19937 rng(0);
  std::uniform_real_distribution<double> u(-1, 1);
  std::vector<SE3> gt_poses;
  for (int i = 0; i < 4; ++i) gt_poses.emplace_back(Eigen::Quaterniond(Eigen::AngleAxisd(0.05 * i, Vec3::UnitY())), Vec3(0.3 * i, 0, 0));
  std::vector<Vec3> gt_pts;
  for (int i = 0; i < 40; ++i) gt_pts.emplace_back(2 * u(rng), 1.5 * u(rng), 4 + 2 * u(rng));

  Problem p;
  for (std::size_t i = 0; i < gt_poses.size(); ++i) {
    SE3 init = gt_poses[i];
    if (i > 0) init = init * SE3::exp((Vec6() << 0.05 * u(rng), 0.05 * u(rng), 0.05 * u(rng), 0.02 * u(rng), 0.02 * u(rng), 0.02 * u(rng)).finished());
    p.addVariable(VariableId{10 + i}, Variable::Pose(init));
  }
  p.setFixed(VariableId{10});
  for (std::size_t j = 0; j < gt_pts.size(); ++j) {
    p.addVariable(VariableId{100 + j}, Variable::Point(gt_pts[j] + Vec3(0.1 * u(rng), 0.1 * u(rng), 0.2 * u(rng))));
    for (std::size_t i = 0; i < gt_poses.size(); ++i) {
      const Vec2 uv = cam.project(gt_poses[i].inverse() * gt_pts[j]) + Vec2(0.2 * u(rng), 0.2 * u(rng));
      auto f = std::make_unique<ReprojectionFactor>(VariableId{10 + i}, VariableId{100 + j}, cam, SE3(), uv, 1.0);
      f->loss = RobustLoss::Huber(std::sqrt(5.991));
      p.addFactor(std::move(f));
    }
  }
  auto solver = makeSolver(mvslam::SolverBackend::Ceres);
  SolveOptions so;
  so.linear_solver = LinearSolverType::SparseSchur;
  so.max_iterations = 30;
  const auto rep = solver->solve(p, so);
  EXPECT_TRUE(rep.converged) << rep.message;
  EXPECT_LT(rep.final_chi2, rep.initial_chi2);
  for (std::size_t i = 1; i < gt_poses.size(); ++i) {
    const Vec6 err = (gt_poses[i].inverse() * p.variable(VariableId{10 + i}).pose).log();
    EXPECT_LT(err.head<3>().norm(), 0.01) << "pose " << i;
    EXPECT_LT(err.tail<3>().norm(), 0.005) << "pose " << i;
  }
}

TEST(CeresSolver, StopTokenPreempts) {
  Problem p;
  p.addVariable(VariableId{1}, Variable::Point(Vec3(5, 5, 5)));
  std::stop_source src;
  src.request_stop();
  auto solver = makeSolver(mvslam::SolverBackend::Ceres);
  SolveOptions so;
  so.linear_solver = LinearSolverType::DenseQr;
  const auto rep = solver->solve(p, so, src.get_token());
  EXPECT_TRUE(rep.preempted || rep.iterations <= 1);
}
