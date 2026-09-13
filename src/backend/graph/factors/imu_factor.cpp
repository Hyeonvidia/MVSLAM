#include "backend/graph/factors/imu_factor.hpp"

#include "core/math/so3_jacobians.hpp"

namespace mvslam::backend::graph {

namespace {
core::Vec3 logRot(const core::Mat3& M) { return core::SO3(Eigen::Quaterniond(M).normalized()).log(); }

// Shared residual/Jacobian core for the two inertial factors. `s` = scale (1 for ImuFactor).
struct InertialTerms {
  core::Mat3 Ri, Rj, dR;
  core::Vec3 er, ev, ep, wv, wp, dbg;
  double dt;
};
InertialTerms inertialTerms(const imu::Preintegration& P, const core::SE3& Ti, const core::Vec3& vi, const core::Vec6& b,
                            const core::SE3& Tj, const core::Vec3& vj, const core::Vec3& g_w, double s) {
  InertialTerms t;
  t.Ri = Ti.rotationMatrix(); t.Rj = Tj.rotationMatrix();
  t.dt = P.dt;
  t.dbg = b.head<3>() - P.bias_lin.head<3>();
  t.dR = P.deltaR(b);
  t.er = logRot(t.dR.transpose() * t.Ri.transpose() * t.Rj);
  t.wv = s * (vj - vi) - g_w * t.dt;
  t.wp = s * (Tj.translation() - Ti.translation() - vi * t.dt) - 0.5 * g_w * t.dt * t.dt;
  t.ev = t.Ri.transpose() * t.wv - P.deltaV(b);
  t.ep = t.Ri.transpose() * t.wp - P.deltaP(b);
  return t;
}
// Jacobians w.r.t. [pose_i(6), vel_i(3), bias_i(6), pose_j(6), vel_j(3)] before whitening.
void inertialJacobians(const imu::Preintegration& P, const InertialTerms& t, double s, std::vector<Eigen::MatrixXd>& J) {
  const core::Mat3 invJr = core::inverseRightJacobianSO3(t.er);
  const core::Mat3 Eer = core::SO3::exp(t.er).matrix();
  const core::Mat3 RiT = t.Ri.transpose();
  Eigen::MatrixXd Jpi = Eigen::MatrixXd::Zero(9, 6), Jvi = Eigen::MatrixXd::Zero(9, 3), Jbi = Eigen::MatrixXd::Zero(9, 6),
                  Jpj = Eigen::MatrixXd::Zero(9, 6), Jvj = Eigen::MatrixXd::Zero(9, 3);
  Jpi.block<3, 3>(0, 3) = -invJr * t.Rj.transpose() * t.Ri;
  Jpi.block<3, 3>(3, 3) = core::SO3::hat(RiT * t.wv);
  Jpi.block<3, 3>(6, 3) = core::SO3::hat(RiT * t.wp);
  Jpi.block<3, 3>(6, 0) = -s * core::Mat3::Identity();
  Jvi.block<3, 3>(3, 0) = -s * RiT;
  Jvi.block<3, 3>(6, 0) = -s * RiT * t.dt;
  Jbi.block<3, 3>(0, 0) = -invJr * Eer.transpose() * core::rightJacobianSO3(P.JRg * t.dbg) * P.JRg;
  Jbi.block<3, 3>(3, 0) = -P.JVg;
  Jbi.block<3, 3>(3, 3) = -P.JVa;
  Jbi.block<3, 3>(6, 0) = -P.JPg;
  Jbi.block<3, 3>(6, 3) = -P.JPa;
  Jpj.block<3, 3>(0, 3) = invJr;
  Jpj.block<3, 3>(6, 0) = s * RiT * t.Rj;
  Jvj.block<3, 3>(3, 0) = s * RiT;
  J = {Jpi, Jvi, Jbi, Jpj, Jvj};
}
}  // namespace

ImuFactor::ImuFactor(core::VariableId pose_i, core::VariableId vel_i, core::VariableId bias_i, core::VariableId pose_j, core::VariableId vel_j,
                     std::shared_ptr<const imu::Preintegration> preint, const core::Vec3& g_w, double info_scale)
    : keys_{pose_i, vel_i, bias_i, pose_j, vel_j}, preint_(std::move(preint)), g_w_(g_w), sqrt_info_(std::sqrt(info_scale) * preint_->sqrtInformation()) {}

bool ImuFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const auto& Ti = values.at(keys_[0]).pose;
  const core::Vec3 vi = values.at(keys_[1]).vec;
  const core::Vec6 b = values.at(keys_[2]).vec;
  const auto& Tj = values.at(keys_[3]).pose;
  const core::Vec3 vj = values.at(keys_[4]).vec;
  const InertialTerms t = inertialTerms(*preint_, Ti, vi, b, Tj, vj, g_w_, 1.0);
  Eigen::Matrix<double, 9, 1> r;
  r << t.er, t.ev, t.ep;
  residual = sqrt_info_ * r;
  if (jacobians) {
    inertialJacobians(*preint_, t, 1.0, *jacobians);
    for (auto& J : *jacobians) J = sqrt_info_ * J;
  }
  return true;
}

ImuScaleGravityFactor::ImuScaleGravityFactor(core::VariableId pose_i, core::VariableId vel_i, core::VariableId bias_i, core::VariableId pose_j,
                                             core::VariableId vel_j, core::VariableId gravity_dir, core::VariableId log_scale,
                                             std::shared_ptr<const imu::Preintegration> preint, double gravity_magnitude)
    : keys_{pose_i, vel_i, bias_i, pose_j, vel_j, gravity_dir, log_scale}, preint_(std::move(preint)), G_(gravity_magnitude),
      sqrt_info_(preint_->sqrtInformation()) {}

bool ImuScaleGravityFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const auto& Ti = values.at(keys_[0]).pose;
  const core::Vec3 vi = values.at(keys_[1]).vec;
  const core::Vec6 b = values.at(keys_[2]).vec;
  const auto& Tj = values.at(keys_[3]).pose;
  const core::Vec3 vj = values.at(keys_[4]).vec;
  const core::Mat3 Rwg = values.at(keys_[5]).rot.matrix();
  const double s = values.at(keys_[6]).scale();
  const core::Vec3 gI(0, 0, -G_);
  const core::Vec3 g_w = Rwg * gI;
  const InertialTerms t = inertialTerms(*preint_, Ti, vi, b, Tj, vj, g_w, s);
  Eigen::Matrix<double, 9, 1> r;
  r << t.er, t.ev, t.ep;
  residual = sqrt_info_ * r;
  if (jacobians) {
    inertialJacobians(*preint_, t, s, *jacobians);
    const core::Mat3 RiT = t.Ri.transpose();
    Eigen::MatrixXd Jg = Eigen::MatrixXd::Zero(9, 2), Js = Eigen::MatrixXd::Zero(9, 1);
    const Eigen::Matrix<double, 3, 2> dg = -Rwg * core::SO3::hat(gI).leftCols<2>();  // d g_w / d [d1 d2]
    Jg.block<3, 2>(3, 0) = -RiT * dg * t.dt;
    Jg.block<3, 2>(6, 0) = -0.5 * RiT * dg * t.dt * t.dt;
    Js.block<3, 1>(3, 0) = RiT * (vj - vi) * s;
    Js.block<3, 1>(6, 0) = RiT * (Tj.translation() - Ti.translation() - vi * t.dt) * s;
    jacobians->push_back(Jg);
    jacobians->push_back(Js);
    for (auto& J : *jacobians) J = sqrt_info_ * J;
  }
  return true;
}

BiasRandomWalkFactor::BiasRandomWalkFactor(core::VariableId bias_i, core::VariableId bias_j, std::shared_ptr<const imu::Preintegration> preint)
    : keys_{bias_i, bias_j}, sqrt_info_(preint->walkSqrtInformation()) {}

bool BiasRandomWalkFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const core::Vec6 bi = values.at(keys_[0]).vec, bj = values.at(keys_[1]).vec;
  residual = sqrt_info_ * (bj - bi);
  if (jacobians) { jacobians->resize(2); (*jacobians)[0] = -sqrt_info_; (*jacobians)[1] = sqrt_info_; }
  return true;
}

BiasPriorFactor::BiasPriorFactor(core::VariableId bias, const core::Vec6& b0, double info_gyro, double info_acc) : keys_{bias}, b0_(b0) {
  sqrt_info_ << core::Vec3::Constant(std::sqrt(std::max(info_gyro, 0.0))), core::Vec3::Constant(std::sqrt(std::max(info_acc, 0.0)));
}

bool BiasPriorFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const core::Vec6 b = values.at(keys_[0]).vec;
  residual = sqrt_info_.cwiseProduct(b - b0_);
  if (jacobians) { jacobians->resize(1); (*jacobians)[0] = sqrt_info_.asDiagonal(); }
  return true;
}

}  // namespace mvslam::backend::graph
