#include <gtest/gtest.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "core/math/so3_jacobians.hpp"
#include "imu_simulation.hpp"
#include "sensors/imu/preintegration.hpp"

using namespace mvslam::core;
using mvslam::sensors::imu::Preintegrator;
using mvslam::testing::ImuTrajectory;

namespace {
mvslam::sensors::ImuCalibration calib() {
  mvslam::sensors::ImuCalibration c;
  c.rate_hz = 200; c.noise_gyro = 1.7e-4; c.noise_acc = 2e-3; c.walk_gyro = 1.9e-5; c.walk_acc = 3e-3; c.gravity = 9.81;
  return c;
}
// Midpoint integration of the analytic measurements between t0 and t1.
void integrateBetween(Preintegrator& pi, const ImuTrajectory& traj, double t0, double t1, double rate) {
  const int n = static_cast<int>(std::round((t1 - t0) * rate));
  const double dt = (t1 - t0) / n;
  for (int i = 0; i < n; ++i) {
    const double ta = t0 + i * dt, tb = ta + dt;
    pi.integrate(0.5 * (traj.acc(ta) + traj.acc(tb)), 0.5 * (traj.gyro(ta) + traj.gyro(tb)), dt);
  }
}
}  // namespace

TEST(Preintegration, MatchesAnalyticRelativeMotion) {
  ImuTrajectory traj;
  Preintegrator pi(calib());
  pi.reset(Vec6::Zero());
  const double t0 = 0.3, t1 = 1.3;
  integrateBetween(pi, traj, t0, t1, 1000);
  const auto& P = pi.state();
  const double dt = t1 - t0;
  EXPECT_NEAR(P.dt, dt, 1e-12);
  const SO3 R0 = traj.R(t0);
  const Mat3 dR_gt = (R0.inverse() * traj.R(t1)).matrix();
  const Vec3 dV_gt = R0.inverse() * (traj.v(t1) - traj.v(t0) - traj.g_w * dt);
  const Vec3 dP_gt = R0.inverse() * (traj.p(t1) - traj.p(t0) - traj.v(t0) * dt - 0.5 * traj.g_w * dt * dt);
  // First-order (ORB-SLAM3) discretization: errors scale with dt; 1 kHz over 1 s with |w|~0.37 rad/s, |a|~2 m/s^2.
  EXPECT_LT((P.dR - dR_gt).norm(), 1e-5);
  EXPECT_LT((P.dV - dV_gt).norm(), 3e-3);
  EXPECT_LT((P.dP - dP_gt).norm(), 2e-3);
  Preintegrator coarse(calib());
  coarse.reset(Vec6::Zero());
  integrateBetween(coarse, traj, t0, t1, 200);
  EXPECT_LT((P.dV - dV_gt).norm(), (coarse.state().dV - dV_gt).norm());  // converges with the rate
  // Covariance: symmetric PSD, growing with time.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 9, 9>> es(P.cov);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-15);
  EXPECT_GT(P.cov.trace(), 0);
  EXPECT_GT(P.cov_walk.trace(), 0);
  Eigen::LLT<Eigen::Matrix<double, 9, 9>> llt(P.information());
  EXPECT_EQ(llt.info(), Eigen::Success);
  const auto W = P.sqrtInformation();
  EXPECT_LT((W.transpose() * W - P.information()).norm() / P.information().norm(), 1e-9);
}

TEST(Preintegration, BiasJacobiansMatchReintegration) {
  ImuTrajectory traj;
  traj.bias << 0.02, -0.01, 0.015, 0.1, -0.05, 0.08;  // measurements carry these biases
  Preintegrator pi(calib());
  pi.reset(Vec6::Zero());  // linearized at zero bias (wrong)
  integrateBetween(pi, traj, 0.0, 0.5, 200);
  const auto lin = pi.state();
  // Numerical bias Jacobians from reintegration.
  const double eps = 1e-6;
  Mat3 JRg_num, JVg_num, JVa_num, JPg_num, JPa_num;
  for (int i = 0; i < 3; ++i) {
    Vec6 bp = Vec6::Zero(), bm = Vec6::Zero();
    bp(i) = eps; bm(i) = -eps;
    Preintegrator a(calib()), b(calib());
    a.reset(Vec6::Zero()); b.reset(Vec6::Zero());
    integrateBetween(a, traj, 0.0, 0.5, 200); integrateBetween(b, traj, 0.0, 0.5, 200);
    a.reintegrate(bp); b.reintegrate(bm);
    JRg_num.col(i) = SO3(Eigen::Quaterniond(lin.dR.transpose() * a.state().dR)).log() / (2 * eps) -
                     SO3(Eigen::Quaterniond(lin.dR.transpose() * b.state().dR)).log() / (2 * eps);
    JVg_num.col(i) = (a.state().dV - b.state().dV) / (2 * eps);
    JPg_num.col(i) = (a.state().dP - b.state().dP) / (2 * eps);
    Vec6 ap = Vec6::Zero(), am = Vec6::Zero();
    ap(3 + i) = eps; am(3 + i) = -eps;
    a.reintegrate(ap); b.reintegrate(am);
    JVa_num.col(i) = (a.state().dV - b.state().dV) / (2 * eps);
    JPa_num.col(i) = (a.state().dP - b.state().dP) / (2 * eps);
  }
  EXPECT_LT((lin.JRg - JRg_num).norm(), 1e-4) << lin.JRg << "\n---\n" << JRg_num;
  EXPECT_LT((lin.JVg - JVg_num).norm(), 1e-4);
  EXPECT_LT((lin.JVa - JVa_num).norm(), 1e-6);
  EXPECT_LT((lin.JPg - JPg_num).norm(), 1e-4);
  EXPECT_LT((lin.JPa - JPa_num).norm(), 1e-6);
  // First-order correction with the true bias approaches the exact reintegration.
  Preintegrator exact(calib());
  exact.reset(traj.bias);
  integrateBetween(exact, traj, 0.0, 0.5, 200);
  EXPECT_LT((lin.deltaR(traj.bias) - exact.state().dR).norm(), 2e-4);
  EXPECT_LT((lin.deltaV(traj.bias) - exact.state().dV).norm(), 2e-3);
  EXPECT_LT((lin.deltaP(traj.bias) - exact.state().dP).norm(), 1e-3);
  // Reintegrating at the true bias reproduces the exact integral.
  Preintegrator re(calib());
  re.reset(Vec6::Zero());
  integrateBetween(re, traj, 0.0, 0.5, 200);
  re.reintegrate(traj.bias);
  EXPECT_LT((re.state().dP - exact.state().dP).norm(), 1e-12);
  EXPECT_LT((re.state().bias_lin - traj.bias).norm(), 1e-15);
}

TEST(Preintegration, MergePreviousEqualsSingleIntegral) {
  ImuTrajectory traj;
  Preintegrator whole(calib()), first(calib()), second(calib());
  for (auto* p : {&whole, &first, &second}) p->reset(Vec6::Zero());
  integrateBetween(whole, traj, 0.0, 1.0, 200);
  integrateBetween(first, traj, 0.0, 0.4, 200);
  integrateBetween(second, traj, 0.4, 1.0, 200);
  second.mergePrevious(first.state());
  EXPECT_NEAR(second.state().dt, whole.state().dt, 1e-12);
  EXPECT_LT((second.state().dP - whole.state().dP).norm(), 1e-12);
  EXPECT_LT((second.state().dR - whole.state().dR).norm(), 1e-12);
  EXPECT_LT((second.state().cov - whole.state().cov).norm(), 1e-15);
  EXPECT_EQ(second.state().measurements.size(), whole.state().measurements.size());
}

TEST(So3Jacobians, RightJacobianIdentities) {
  const Vec3 phi(0.3, -0.2, 0.5), d(1e-6, -2e-6, 1.5e-6);
  const Mat3 Jr = rightJacobianSO3(phi);
  EXPECT_LT((Jr * inverseRightJacobianSO3(phi) - Mat3::Identity()).norm(), 1e-12);
  // Exp(phi + d) ~= Exp(phi) Exp(Jr d)
  const Mat3 lhs = SO3::exp(phi + d).matrix(), rhs = (SO3::exp(phi) * SO3::exp(Jr * d)).matrix();
  EXPECT_LT((lhs - rhs).norm(), 1e-11);
  EXPECT_LT((rightJacobianSO3(Vec3::Zero()) - Mat3::Identity()).norm(), 1e-15);
}
