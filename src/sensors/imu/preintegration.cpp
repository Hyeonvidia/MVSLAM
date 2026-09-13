#include "sensors/imu/preintegration.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "core/math/so3_jacobians.hpp"

namespace mvslam::sensors::imu {

namespace {
template <int N>
Eigen::Matrix<double, N, N> floorEigenvalues(const Eigen::Matrix<double, N, N>& M, double floor_val) {
  const Eigen::Matrix<double, N, N> S = 0.5 * (M + M.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(S);
  Eigen::Matrix<double, N, 1> lam = es.eigenvalues();
  for (int i = 0; i < N; ++i) lam(i) = std::max(lam(i), floor_val);
  return es.eigenvectors() * lam.asDiagonal() * es.eigenvectors().transpose();
}
template <int N>
Eigen::Matrix<double, N, N> sqrtInfoFromCov(const Eigen::Matrix<double, N, N>& cov, double floor_val) {
  const Eigen::Matrix<double, N, N> info = floorEigenvalues<N>(cov, floor_val).inverse();
  const Eigen::Matrix<double, N, N> info_s = 0.5 * (info + info.transpose());
  Eigen::LLT<Eigen::Matrix<double, N, N>> llt(info_s);
  if (llt.info() != Eigen::Success) {  // fall back to the symmetric square root
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, N, N>> es(info_s);
    Eigen::Matrix<double, N, 1> lam = es.eigenvalues();
    for (int i = 0; i < N; ++i) lam(i) = std::sqrt(std::max(lam(i), 0.0));
    return es.eigenvectors() * lam.asDiagonal() * es.eigenvectors().transpose();
  }
  return Eigen::Matrix<double, N, N>(llt.matrixU());  // W = U with U^T U = info
}
}  // namespace

core::Mat3 Preintegration::deltaR(const core::Vec6& bias) const {
  const core::Vec3 dbg = bias.head<3>() - bias_lin.head<3>();
  return core::normalizedRotation(dR * core::SO3::exp(JRg * dbg).matrix());
}
core::Vec3 Preintegration::deltaV(const core::Vec6& bias) const {
  const core::Vec3 dbg = bias.head<3>() - bias_lin.head<3>(), dba = bias.tail<3>() - bias_lin.tail<3>();
  return dV + JVg * dbg + JVa * dba;
}
core::Vec3 Preintegration::deltaP(const core::Vec6& bias) const {
  const core::Vec3 dbg = bias.head<3>() - bias_lin.head<3>(), dba = bias.tail<3>() - bias_lin.tail<3>();
  return dP + JPg * dbg + JPa * dba;
}
Eigen::Matrix<double, 9, 9> Preintegration::information() const { return floorEigenvalues<9>(cov, 1e-12).inverse(); }
Eigen::Matrix<double, 9, 9> Preintegration::sqrtInformation() const { return sqrtInfoFromCov<9>(cov, 1e-12); }
Eigen::Matrix<double, 6, 6> Preintegration::walkSqrtInformation() const { return sqrtInfoFromCov<6>(cov_walk, 1e-14); }

Preintegrator::Preintegrator(const ImuCalibration& calib) : calib_(calib) {
  // Continuous densities -> discrete: measurement noise sigma_d^2 = sigma_c^2 * f, random walk per
  // step sigma_w^2 / f (ORB-SLAM3 IMU::Calib with ng*sqrt(f), ngw/sqrt(f)).
  const double f = calib.rate_hz > 0 ? calib.rate_hz : 200.0;
  noise_.setZero();
  noise_.block<3, 3>(0, 0) = core::Mat3::Identity() * (calib.noise_gyro * calib.noise_gyro * f);
  noise_.block<3, 3>(3, 3) = core::Mat3::Identity() * (calib.noise_acc * calib.noise_acc * f);
  noise_walk_.setZero();
  noise_walk_.block<3, 3>(0, 0) = core::Mat3::Identity() * (calib.walk_gyro * calib.walk_gyro / f);
  noise_walk_.block<3, 3>(3, 3) = core::Mat3::Identity() * (calib.walk_acc * calib.walk_acc / f);
  reset(core::Vec6::Zero());
}

void Preintegrator::reset(const core::Vec6& bias) {
  state_ = Preintegration{};
  state_.bias_lin = bias;
}

void Preintegrator::integrate(const core::Vec3& acc, const core::Vec3& gyro, double dt) {
  if (dt <= 0) return;
  state_.measurements.push_back({acc, gyro, dt});
  integrateInternal(acc, gyro, dt);
}

void Preintegrator::integrateInternal(const core::Vec3& acc_m, const core::Vec3& gyro_m, double dt) {
  auto& s = state_;
  const core::Vec3 acc = acc_m - s.bias_lin.tail<3>();
  const core::Vec3 gyro = gyro_m - s.bias_lin.head<3>();
  const core::Mat3 Wacc = core::SO3::hat(acc);
  // Position and velocity first (with the rotation before this step), as in ORB-SLAM3.
  s.dP += s.dV * dt + 0.5 * s.dR * acc * dt * dt;
  s.dV += s.dR * acc * dt;
  Eigen::Matrix<double, 9, 9> A = Eigen::Matrix<double, 9, 9>::Identity();
  Eigen::Matrix<double, 9, 6> B = Eigen::Matrix<double, 9, 6>::Zero();
  A.block<3, 3>(3, 0) = -s.dR * dt * Wacc;
  A.block<3, 3>(6, 0) = -0.5 * s.dR * dt * dt * Wacc;
  A.block<3, 3>(6, 3) = core::Mat3::Identity() * dt;
  B.block<3, 3>(3, 3) = s.dR * dt;
  B.block<3, 3>(6, 3) = 0.5 * s.dR * dt * dt;
  // Bias Jacobians (position/velocity part uses the previous rotation Jacobian).
  s.JPa = s.JPa + s.JVa * dt - 0.5 * s.dR * dt * dt;
  s.JPg = s.JPg + s.JVg * dt - 0.5 * s.dR * dt * dt * Wacc * s.JRg;
  s.JVa = s.JVa - s.dR * dt;
  s.JVg = s.JVg - s.dR * dt * Wacc * s.JRg;
  // Rotation update.
  const core::Vec3 phi = gyro * dt;
  const core::Mat3 dRi = core::SO3::exp(phi).matrix();
  const core::Mat3 Jr = core::rightJacobianSO3(phi);
  s.dR = core::normalizedRotation(s.dR * dRi);
  A.block<3, 3>(0, 0) = dRi.transpose();
  B.block<3, 3>(0, 0) = Jr * dt;
  s.cov = A * s.cov * A.transpose() + B * noise_ * B.transpose();
  s.cov_walk += noise_walk_;
  s.JRg = dRi.transpose() * s.JRg - Jr * dt;
  s.dt += dt;
}

void Preintegrator::reintegrate(const core::Vec6& new_bias) {
  const auto meas = state_.measurements;
  reset(new_bias);
  for (const auto& m : meas) integrate(m.acc, m.gyro, m.dt);
}

void Preintegrator::mergePrevious(const Preintegration& prev) {
  std::vector<Measurement> meas = prev.measurements;
  meas.insert(meas.end(), state_.measurements.begin(), state_.measurements.end());
  const core::Vec6 bias = state_.bias_lin;
  reset(bias);
  for (const auto& m : meas) integrate(m.acc, m.gyro, m.dt);
}

}  // namespace mvslam::sensors::imu
