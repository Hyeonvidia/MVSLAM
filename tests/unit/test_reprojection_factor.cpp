#include <gtest/gtest.h>

#include "backend/graph/factors/reprojection_factor.hpp"
#include "numeric_jacobian.hpp"
#include "sensors/camera_model.hpp"

using namespace mvslam::core;
using namespace mvslam::backend::graph;

TEST(ReprojectionFactor, AnalyticJacobiansMatchNumeric) {
  mvslam::CameraConfig cc; cc.width = 640; cc.height = 480; cc.intrinsics = {500, 500, 320, 240};
  mvslam::sensors::PinholeCamera cam(cc);
  const SE3 T_cb(Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Vec3(0, 1, 0))), Vec3(0.1, 0, 0.05));
  const SE3 T_wb(Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Vec3(1, 0.2, 0).normalized())), Vec3(1, 2, 3));
  const Vec3 X_w = T_wb * (T_cb.inverse() * Vec3(0.4, -0.3, 3.0));
  const Vec2 uv = cam.project(T_cb * (T_wb.inverse() * X_w)) + Vec2(0.7, -0.4);
  Values v;
  v[VariableId{1}] = Variable::Pose(T_wb);
  v[VariableId{2}] = Variable::Point(X_w);
  ReprojectionFactor f(VariableId{1}, VariableId{2}, cam, T_cb, uv, 4.0);
  Eigen::VectorXd r(2);
  std::vector<Eigen::MatrixXd> J;
  ASSERT_TRUE(f.evaluate(v, r, &J));
  EXPECT_NEAR(r.norm(), 2.0 * std::hypot(0.7, 0.4), 1e-9);  // sqrt(info)=2
  const auto Jn0 = mvslam::testing::numericJacobian(f, v, VariableId{1});
  const auto Jn1 = mvslam::testing::numericJacobian(f, v, VariableId{2});
  EXPECT_LT((J[0] - Jn0).norm() / Jn0.norm(), 1e-6);
  EXPECT_LT((J[1] - Jn1).norm() / Jn1.norm(), 1e-6);
}

TEST(ReprojectionFactor, RejectsPointsBehindCamera) {
  mvslam::CameraConfig cc; cc.width = 640; cc.height = 480; cc.intrinsics = {500, 500, 320, 240};
  mvslam::sensors::PinholeCamera cam(cc);
  Values v;
  v[VariableId{1}] = Variable::Pose(SE3());
  v[VariableId{2}] = Variable::Point(Vec3(0, 0, -1));
  ReprojectionFactor f(VariableId{1}, VariableId{2}, cam, SE3(), Vec2(320, 240));
  Eigen::VectorXd r(2);
  EXPECT_FALSE(f.evaluate(v, r, nullptr));
  EXPECT_FALSE(f.depthPositive(v));
}
