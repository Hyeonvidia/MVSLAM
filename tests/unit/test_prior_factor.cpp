#include <gtest/gtest.h>

#include "backend/graph/factors/prior_factor.hpp"
#include "numeric_jacobian.hpp"

using namespace mvslam::core;
using namespace mvslam::backend::graph;

TEST(PriorFactor, ReproducesQuadraticCostAndJacobian) {
  Values x0;
  x0[VariableId{1}] = Variable::Pose(SE3(Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Vec3::UnitZ())), Vec3(1, 0, 0)));
  x0[VariableId{2}] = Variable::Velocity(Vec3(0.5, 0, 0));
  Eigen::MatrixXd A = Eigen::MatrixXd::Random(9, 9);
  Eigen::MatrixXd H = A * A.transpose() + Eigen::MatrixXd::Identity(9, 9);
  Eigen::VectorXd g = Eigen::VectorXd::Random(9);
  PriorFactor f({VariableId{1}, VariableId{2}}, x0, H, g);
  EXPECT_EQ(f.rank(), 9);
  // Perturb and compare 1/2||r||^2 - const with g^T d + 1/2 d^T H d.
  Values x = x0;
  Eigen::VectorXd d(9); d << 0.01, -0.02, 0.03, 0.004, -0.005, 0.006, 0.1, -0.1, 0.05;
  x.at(VariableId{1}).plus(d.head<6>());
  x.at(VariableId{2}).plus(d.tail<3>());
  Eigen::VectorXd r0(9), r(9);
  f.evaluate(x0, r0, nullptr);
  f.evaluate(x, r, nullptr);
  const double lhs = 0.5 * r.squaredNorm() - 0.5 * r0.squaredNorm();
  const double rhs = g.dot(d) + 0.5 * d.dot(H * d);
  EXPECT_NEAR(lhs, rhs, 1e-9);
  std::vector<Eigen::MatrixXd> J;
  f.evaluate(x0, r0, &J);
  const auto Jn = mvslam::testing::numericJacobian(f, x0, VariableId{1});
  EXPECT_LT((J[0] - Jn).norm() / Jn.norm(), 1e-5);
}

TEST(PriorFactor, HandlesRankDeficientInformation) {
  Values x0;
  x0[VariableId{1}] = Variable::Point(Vec3::Zero());
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
  H(0, 0) = 4.0;  // only x observed
  PriorFactor f({VariableId{1}}, x0, H, Eigen::VectorXd::Zero(3));
  EXPECT_EQ(f.rank(), 1);
  Values x = x0;
  x.at(VariableId{1}).plus(Vec3(0.5, 10, 10));  // unobserved directions carry no cost
  Eigen::VectorXd r(3);
  f.evaluate(x, r, nullptr);
  EXPECT_NEAR(0.5 * r.squaredNorm(), 0.5 * 4.0 * 0.25, 1e-12);
}
