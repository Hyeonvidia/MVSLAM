#pragma once
// Pose-graph and loop-verification factors (docs/design/03-backend.md §1.2, §5).
//  * RelativePoseSE3Factor: r = W log(T_ij^-1 T_i^-1 T_j) between two body poses (essential graph,
//    OV2SLAM LeftSE3RelativePoseError / ORB EdgeSE3), exact right-perturbation Jacobians via Sophus.
//  * LoopReprojectionFactor: reprojection of a point of one map into a keyframe of the other map through
//    the loop transform T (the only variable): p = T_cw_obs * T * X (forward) or T_cw_obs * T^-1 * X.
#include "backend/graph/factor.hpp"
#include "sensors/camera_model.hpp"

namespace mvslam::backend::graph {

class RelativePoseSE3Factor final : public IFactor {
 public:
  RelativePoseSE3Factor(core::VariableId pose_i, core::VariableId pose_j, const core::SE3& T_ij, double info = 1.0)
      : keys_{pose_i, pose_j}, T_ij_inv_(T_ij.inverse()), sqrt_info_(std::sqrt(info)) {}
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 6; }
  std::string name() const override { return "RelativePoseSE3"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;

 private:
  std::vector<core::VariableId> keys_;
  core::SE3 T_ij_inv_;
  double sqrt_info_;
};

class LoopReprojectionFactor final : public IFactor {
 public:
  // `inverse` = false: p_c = T_cw_obs * T * X;  true: p_c = T_cw_obs * T^-1 * X.
  LoopReprojectionFactor(core::VariableId transform, const sensors::ICameraModel& cam, const core::SE3& T_cw_obs, const core::Vec3& X,
                         const core::Vec2& uv, bool inverse, double info = 1.0)
      : keys_{transform}, cam_(cam), T_cw_(T_cw_obs), X_(X), uv_(uv), inverse_(inverse), sqrt_info_(std::sqrt(info)) {}
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 2; }
  std::string name() const override { return "LoopReprojection"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual, std::vector<Eigen::MatrixXd>* jacobians) const override;
  bool depthPositive(const Values& values) const;

 private:
  std::vector<core::VariableId> keys_;
  const sensors::ICameraModel& cam_;
  core::SE3 T_cw_;
  core::Vec3 X_;
  core::Vec2 uv_;
  bool inverse_;
  double sqrt_info_;
};

}  // namespace mvslam::backend::graph
