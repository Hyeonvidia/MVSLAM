#pragma once
// Inertial factors (ORB-SLAM3 EdgeInertial / EdgeInertialGS / EdgeGyroRW+EdgeAccRW / EdgePriorGyro+Acc
// formulation, docs/design/03-backend.md §1.2), analytic Jacobians for the right-multiplicative body
// pose [rho, phi], additive velocity/bias, GravityDir R_wg * exp([d1 d2 0]), LogScale s = exp(sigma).
// Residuals are whitened with the preintegration's 9x9 information.
#include "backend/graph/factor.hpp"
#include "backend/imu/preintegrator.hpp"

namespace mvslam::backend::graph {

// r = W [Log(dR(b)^T R_i^T R_j); R_i^T (v_j - v_i - g dt) - dV(b); R_i^T (p_j - p_i - v_i dt - 1/2 g dt^2) - dP(b)]
class ImuFactor final : public IFactor {
 public:
  ImuFactor(core::VariableId pose_i, core::VariableId vel_i, core::VariableId bias_i, core::VariableId pose_j,
            core::VariableId vel_j, std::shared_ptr<const imu::Preintegration> preint, const core::Vec3& g_w, double info_scale = 1.0);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 9; }
  std::string name() const override { return "Imu"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;
  const imu::Preintegration& preintegration() const { return *preint_; }

 private:
  std::vector<core::VariableId> keys_;
  std::shared_ptr<const imu::Preintegration> preint_;
  core::Vec3 g_w_;
  Eigen::Matrix<double, 9, 9> sqrt_info_;
};

// Initialization variant: gravity direction and (log) scale are variables, g = R_wg [0 0 -G]^T and
// the map's translations/velocities are scaled by s: ev = R_i^T (s (v_j - v_i) - g dt) - dV, ...
class ImuScaleGravityFactor final : public IFactor {
 public:
  ImuScaleGravityFactor(core::VariableId pose_i, core::VariableId vel_i, core::VariableId bias_i, core::VariableId pose_j,
                        core::VariableId vel_j, core::VariableId gravity_dir, core::VariableId log_scale,
                        std::shared_ptr<const imu::Preintegration> preint, double gravity_magnitude);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 9; }
  std::string name() const override { return "ImuScaleGravity"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;

 private:
  std::vector<core::VariableId> keys_;
  std::shared_ptr<const imu::Preintegration> preint_;
  double G_;
  Eigen::Matrix<double, 9, 9> sqrt_info_;
};

// r = W (b_j - b_i), W from the preintegration's random-walk covariance over the interval.
class BiasRandomWalkFactor final : public IFactor {
 public:
  BiasRandomWalkFactor(core::VariableId bias_i, core::VariableId bias_j, std::shared_ptr<const imu::Preintegration> preint);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 6; }
  std::string name() const override { return "BiasRandomWalk"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;

 private:
  std::vector<core::VariableId> keys_;
  Eigen::Matrix<double, 6, 6> sqrt_info_;
};

// r = [sqrt(info_g) (bg - bg0); sqrt(info_a) (ba - ba0)] — the staged initialization priors.
class BiasPriorFactor final : public IFactor {
 public:
  BiasPriorFactor(core::VariableId bias, const core::Vec6& b0, double info_gyro, double info_acc);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 6; }
  std::string name() const override { return "BiasPrior"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;

 private:
  std::vector<core::VariableId> keys_;
  core::Vec6 b0_, sqrt_info_;
};

}  // namespace mvslam::backend::graph
