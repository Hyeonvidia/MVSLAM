#pragma once
// 2-D reprojection of a world point through rig camera `cam` of body pose T_wb.
// r = sqrt_info * (proj(T_cb * T_wb^{-1} * X) - uv). Analytic Jacobians for right-perturbed pose.
#include "backend/graph/factor.hpp"
#include "sensors/camera_model.hpp"

namespace mvslam::backend::graph {

class ReprojectionFactor final : public IFactor {
 public:
  ReprojectionFactor(core::VariableId pose, core::VariableId point, const sensors::ICameraModel& cam,
                     const core::SE3& T_cb, const core::Vec2& uv, double info = 1.0)
      : keys_{pose, point}, cam_(cam), T_cb_(T_cb), uv_(uv), sqrt_info_(std::sqrt(info)) {}
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 2; }
  std::string name() const override { return "Reprojection"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                std::vector<Eigen::MatrixXd>* jacobians) const override;
  bool depthPositive(const Values& values) const;

 private:
  std::vector<core::VariableId> keys_;
  const sensors::ICameraModel& cam_;
  core::SE3 T_cb_;
  core::Vec2 uv_;
  double sqrt_info_;
};

}  // namespace mvslam::backend::graph
