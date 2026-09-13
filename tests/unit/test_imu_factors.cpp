#include <gtest/gtest.h>

#include <random>

#include "backend/graph/factors/imu_factor.hpp"
#include "imu_simulation.hpp"
#include "numeric_jacobian.hpp"

using namespace mvslam::core;
using namespace mvslam::backend::graph;
using mvslam::testing::ImuTrajectory;
using mvslam::testing::numericJacobian;

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
void expectJacobians(const IFactor& f, const Values& v, double tol) {
  Eigen::VectorXd r(f.residualDim());
  std::vector<Eigen::MatrixXd> J;
  ASSERT_TRUE(f.evaluate(v, r, &J));
  ASSERT_EQ(J.size(), f.keys().size());
  for (std::size_t k = 0; k < f.keys().size(); ++k) {
    const Eigen::MatrixXd Jn = numericJacobian(f, v, f.keys()[k], 1e-6);
    const double scale = std::max(1.0, Jn.norm());
    EXPECT_LT((J[k] - Jn).norm() / scale, tol) << f.name() << " key " << k << "\nanalytic\n" << J[k] << "\nnumeric\n" << Jn;
  }
}
const VariableId kPi{1}, kVi{2}, kBi{3}, kPj{4}, kVj{5}, kG{6}, kS{7}, kBj{8};
}  // namespace

TEST(ImuFactor, ResidualVanishesOnTruthAndJacobiansMatchNumeric) {
  ImuTrajectory traj;
  traj.bias << 0.01, -0.02, 0.005, 0.05, 0.02, -0.03;
  const double t0 = 0.2, t1 = 0.7;
  auto P = preintegrate(traj, t0, t1, traj.bias);
  Values v;
  v[kPi] = Variable::Pose(traj.T(t0)); v[kVi] = Variable::Velocity(traj.v(t0)); v[kBi] = Variable::Bias(traj.bias);
  v[kPj] = Variable::Pose(traj.T(t1)); v[kVj] = Variable::Velocity(traj.v(t1));
  ImuFactor f(kPi, kVi, kBi, kPj, kVj, P, traj.g_w);
  Eigen::VectorXd r(9);
  ASSERT_TRUE(f.evaluate(v, r, nullptr));
  // Whitened residual of the exact trajectory: tiny compared with the measurement noise.
  Eigen::Matrix<double, 9, 1> raw = P->sqrtInformation().inverse() * r;
  EXPECT_LT(raw.head<3>().norm(), 1e-4);
  EXPECT_LT(raw.segment<3>(3).norm(), 1e-2);  // 200 Hz first-order integration error over 0.5 s
  EXPECT_LT(raw.tail<3>().norm(), 5e-3);
  // Perturbed states (non-zero residual) with a bias offset from the linearization point.
  std::mt19937 rng(4);
  std::uniform_real_distribution<double> u(-1, 1);
  Vec6 d; for (int i = 0; i < 6; ++i) d(i) = 0.05 * u(rng);
  v[kPi].plus(d); for (int i = 0; i < 6; ++i) d(i) = 0.05 * u(rng); v[kPj].plus(d);
  v[kVi].vec += Vec3(0.1 * u(rng), 0.1 * u(rng), 0.1 * u(rng));
  v[kBi].vec += (Vec6() << 0.003, -0.002, 0.004, 0.02, -0.01, 0.03).finished();
  expectJacobians(f, v, 1e-5);
}

TEST(ImuScaleGravityFactor, JacobiansMatchNumericIncludingScaleAndGravity) {
  ImuTrajectory traj;
  traj.bias << -0.01, 0.02, 0.01, 0.03, -0.04, 0.02;
  const double t0 = 1.0, t1 = 1.5;
  auto P = preintegrate(traj, t0, t1, Vec6::Zero());
  // A map expressed in an arbitrarily rotated, 1/s-scaled frame: T_map = T_mw * T with translations scaled.
  const double s = 2.5;
  const SO3 R_mw = SO3::exp(Vec3(0.4, -0.3, 0.2));
  auto toMap = [&](const SE3& T) { return SE3(R_mw * T.so3(), (R_mw * T.translation()) / s); };
  Values v;
  v[kPi] = Variable::Pose(toMap(traj.T(t0))); v[kVi] = Variable::Velocity((R_mw * traj.v(t0)) / s); v[kBi] = Variable::Bias(traj.bias);
  v[kPj] = Variable::Pose(toMap(traj.T(t1))); v[kVj] = Variable::Velocity((R_mw * traj.v(t1)) / s);
  v[kG] = Variable::GravityDir(R_mw);   // gravity in the map frame = R_mw * (0,0,-G)
  v[kS] = Variable::LogScale(std::log(s));
  ImuScaleGravityFactor f(kPi, kVi, kBi, kPj, kVj, kG, kS, P, 9.81);
  Eigen::VectorXd r(9);
  ASSERT_TRUE(f.evaluate(v, r, nullptr));
  Eigen::Matrix<double, 9, 1> raw = P->sqrtInformation().inverse() * r;
  EXPECT_LT(raw.norm(), 3e-3);  // exact geometry with first-order bias correction
  v[kG].plus(Eigen::Vector2d(0.05, -0.03));
  v[kS].vec(0) += 0.1;
  v[kVi].vec += Vec3(0.05, -0.02, 0.03);
  expectJacobians(f, v, 1e-5);
}

TEST(BiasFactors, JacobiansAndResiduals) {
  ImuTrajectory traj;
  auto P = preintegrate(traj, 0.0, 0.5, Vec6::Zero());
  Values v;
  v[kBi] = Variable::Bias((Vec6() << 0.01, 0.02, 0.03, 0.1, 0.2, 0.3).finished());
  v[kBj] = Variable::Bias((Vec6() << 0.011, 0.019, 0.03, 0.12, 0.2, 0.29).finished());
  BiasRandomWalkFactor rw(kBi, kBj, P);
  expectJacobians(rw, v, 1e-8);
  Eigen::VectorXd r(6);
  rw.evaluate(v, r, nullptr);
  EXPECT_GT(r.norm(), 0);
  v[kBj].vec = v[kBi].vec;
  rw.evaluate(v, r, nullptr);
  EXPECT_LT(r.norm(), 1e-12);
  BiasPriorFactor prior(kBi, Vec6::Zero(), 1e2, 1e10);
  expectJacobians(prior, v, 1e-8);
  prior.evaluate(v, r, nullptr);
  EXPECT_NEAR(r(0), std::sqrt(1e2) * 0.01, 1e-12);
  EXPECT_NEAR(r(3), std::sqrt(1e10) * 0.1, 1e-6);
}
