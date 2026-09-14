#include "backend/graph/factors/relative_pose_factor.hpp"

namespace mvslam::backend::graph {

bool RelativePoseSE3Factor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const core::SE3& Ti = values.at(keys_[0]).pose;
  const core::SE3& Tj = values.at(keys_[1]).pose;
  const core::SE3 E = T_ij_inv_ * Ti.inverse() * Tj;
  const core::Vec6 xi = E.log();
  residual = sqrt_info_ * xi;
  if (jacobians) {
    jacobians->resize(2);
    const Eigen::Matrix<double, 6, 6> Jl_inv = core::SE3::leftJacobianInverse(xi);     // log(exp(a) E) ~ xi + Jl^-1 a
    const Eigen::Matrix<double, 6, 6> Jr_inv = core::SE3::leftJacobianInverse(-xi);    // log(E exp(d)) ~ xi + Jr^-1 d
    (*jacobians)[0] = -sqrt_info_ * Jl_inv * T_ij_inv_.Adj();
    (*jacobians)[1] = sqrt_info_ * Jr_inv;
  }
  return true;
}

bool LoopReprojectionFactor::depthPositive(const Values& values) const {
  const core::SE3& T = values.at(keys_[0]).pose;
  const core::Vec3 p_c = T_cw_ * (inverse_ ? T.inverse() * X_ : T * X_);
  return p_c.z() > 1e-6;
}

bool LoopReprojectionFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const {
  const core::SE3& T = values.at(keys_[0]).pose;
  const core::Vec3 Y = inverse_ ? T.inverse() * X_ : T * X_;   // point in the observing map's world frame
  const core::Vec3 p_c = T_cw_ * Y;
  if (p_c.z() <= 1e-6) return false;
  residual = sqrt_info_ * (cam_.project(p_c) - uv_);
  if (jacobians) {
    const Eigen::Matrix<double, 2, 3> Jp = sqrt_info_ * cam_.projectJacobian(p_c);
    const core::Mat3 R_cw = T_cw_.rotationMatrix();
    Eigen::Matrix<double, 3, 6> dY;
    if (!inverse_) {  // T exp(d) X ~ T (X + rho + phi x X)
      dY.leftCols<3>() = T.rotationMatrix();
      dY.rightCols<3>() = -T.rotationMatrix() * core::SO3::hat(X_);
    } else {          // exp(-d) T^-1 X ~ Y - rho - phi x Y
      dY.leftCols<3>() = -core::Mat3::Identity();
      dY.rightCols<3>() = core::SO3::hat(Y);
    }
    jacobians->resize(1);
    (*jacobians)[0] = Jp * R_cw * dY;
  }
  return true;
}

}  // namespace mvslam::backend::graph
