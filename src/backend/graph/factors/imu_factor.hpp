#pragma once
// IMU preintegration factor (ORB-SLAM3 EdgeInertial formulation, docs/design/03-backend.md §1.2).
// Keys: pose_i, vel_i, bias_i(6), pose_j, vel_j. Residual [dR, dV, dP] (9). Implementation: P2.
#include "backend/graph/factor.hpp"
#include "backend/imu/preintegrator.hpp"

namespace mvslam::backend::graph {

class ImuFactor final : public IFactor {
 public:
  ImuFactor(core::VariableId pose_i, core::VariableId vel_i, core::VariableId bias_i, core::VariableId pose_j,
            core::VariableId vel_j, std::shared_ptr<const imu::Preintegration> preint, double gravity);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 9; }
  std::string name() const override { return "Imu"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                std::vector<Eigen::MatrixXd>* jacobians) const override;
 private:
  std::vector<core::VariableId> keys_;
  std::shared_ptr<const imu::Preintegration> preint_;
  core::Vec3 g_w_;
  Eigen::Matrix<double, 9, 9> sqrt_info_;
};

}  // namespace mvslam::backend::graph
