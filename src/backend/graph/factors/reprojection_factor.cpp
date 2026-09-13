#include "backend/graph/factors/reprojection_factor.hpp"

namespace mvslam::backend::graph {

bool ReprojectionFactor::depthPositive(const Values& values) const {
  const core::SE3& T_wb = values.at(keys_[0]).pose;
  const core::Vec3 X = values.at(keys_[1]).vec;
  return (T_cb_ * (T_wb.inverse() * X)).z() > 1e-6;
}

bool ReprojectionFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                                  std::vector<Eigen::MatrixXd>* jacobians) const {
  const core::SE3& T_wb = values.at(keys_[0]).pose;
  const core::Vec3 X = values.at(keys_[1]).vec;
  const core::SE3 T_bw = T_wb.inverse();
  const core::Vec3 p_b = T_bw * X;
  const core::Vec3 p_c = T_cb_ * p_b;
  if (p_c.z() <= 1e-6) return false;
  residual = sqrt_info_ * (cam_.project(p_c) - uv_);
  if (jacobians) {
    const Eigen::Matrix<double, 2, 3> Jp = sqrt_info_ * cam_.projectJacobian(p_c);
    const core::Mat3 R_cb = T_cb_.rotationMatrix();
    Eigen::Matrix<double, 3, 6> dpb_ddelta;  // right perturbation T_wb * exp([rho, phi])
    dpb_ddelta.leftCols<3>() = -core::Mat3::Identity();
    dpb_ddelta.rightCols<3>() = core::SO3::hat(p_b);
    jacobians->resize(2);
    (*jacobians)[0] = Jp * R_cb * dpb_ddelta;                 // 2 x 6
    (*jacobians)[1] = Jp * R_cb * T_bw.rotationMatrix();      // 2 x 3
  }
  return true;
}

}  // namespace mvslam::backend::graph
