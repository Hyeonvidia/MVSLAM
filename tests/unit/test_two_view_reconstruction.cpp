#include <gtest/gtest.h>

#include <random>

#include "frontend/opencv/two_view_reconstruction.hpp"
#include "sensors/camera_model.hpp"

using namespace mvslam::core;
using mvslam::frontend::cv_impl::reconstructTwoView;
using mvslam::frontend::cv_impl::TwoViewMatch;
using mvslam::frontend::cv_impl::TwoViewOptions;

namespace {
std::unique_ptr<mvslam::sensors::ICameraModel> camera() {
  mvslam::CameraConfig cc; cc.width = 752; cc.height = 480; cc.intrinsics = {458.654, 457.296, 367.215, 248.375};
  return mvslam::sensors::makeCameraModel(cc);
}
struct Synthetic { std::vector<TwoViewMatch> matches; std::vector<Vec3> X_c1; std::vector<bool> outlier; };
Synthetic project(const std::vector<Vec3>& X, const SE3& T_c2c1, const mvslam::sensors::ICameraModel& cam, std::mt19937& rng, double noise_px,
                  double outlier_ratio) {
  Synthetic s;
  std::normal_distribution<double> n(0, noise_px);
  std::uniform_real_distribution<double> u(0, 1);
  for (std::size_t i = 0; i < X.size(); ++i) {
    const Vec3 X2 = T_c2c1 * X[i];
    if (X[i].z() <= 0 || X2.z() <= 0) continue;
    Vec2 p1 = cam.project(X[i]) + Vec2(n(rng), n(rng)), p2 = cam.project(X2) + Vec2(n(rng), n(rng));
    const bool out = u(rng) < outlier_ratio;
    if (out) p2 += Vec2(30 + 100 * u(rng), -20 - 80 * u(rng));
    if (!cam.inImage(p1) || !cam.inImage(p2)) continue;
    s.matches.push_back({TrackId{i + 1}, p1, p2});
    s.X_c1.push_back(X[i]);
    s.outlier.push_back(out);
  }
  return s;
}
void expectPose(const SE3& est, const SE3& gt, double rot_deg, double dir_dot) {
  const double ang = (gt.so3().inverse() * est.so3()).log().norm() * 180.0 / M_PI;
  EXPECT_LT(ang, rot_deg);
  EXPECT_GT(est.translation().normalized().dot(gt.translation().normalized()), dir_dot);
  EXPECT_NEAR(est.translation().norm(), 1.0, 1e-9);  // unit-norm translation
}
}  // namespace

TEST(TwoViewReconstruction, GeneralSceneUsesFundamentalMatrix) {
  auto cam = camera();
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1, 1);
  std::vector<Vec3> X;
  for (int i = 0; i < 300; ++i) X.emplace_back(2.0 * u(rng), 1.3 * u(rng), 4.0 + 2.0 * u(rng));
  const SE3 gt(SO3::exp(Vec3(0.01, -0.04, 0.02)), Vec3(-0.80, 0.05, 0.10));  // ~9 deg parallax at 5 m
  const auto s = project(X, gt, *cam, rng, 0.2, 0.10);
  ASSERT_GT(s.matches.size(), 200u);
  auto r = reconstructTwoView(s.matches, *cam, TwoViewOptions{});
  ASSERT_TRUE(r.has_value());
  EXPECT_FALSE(r->used_homography);
  EXPECT_LT(r->r_h, 0.45);
  expectPose(r->T_c2c1, gt, 0.3, 0.999);
  int good = 0, wrong_outliers = 0;
  const double scale = gt.translation().norm();
  for (std::size_t i = 0; i < s.matches.size(); ++i) {
    if (!r->triangulated[i]) continue;
    ++good;
    if (s.outlier[i]) { ++wrong_outliers; continue; }
    // Triangulation is consistent with the recovered pose in both views ...
    EXPECT_LT((cam->project(r->X_c1[i]) - s.matches[i].p1).norm(), 1.0) << i;
    EXPECT_LT((cam->project(r->T_c2c1 * r->X_c1[i]) - s.matches[i].p2).norm(), 1.0) << i;
    // ... and close to the ground truth once the (unit-norm) scale is restored.
    EXPECT_LT((r->X_c1[i] * scale - s.X_c1[i]).norm(), 0.15) << i;
  }
  EXPECT_GE(good, static_cast<int>(0.8 * s.matches.size()));
  EXPECT_LE(wrong_outliers, 2);
}

TEST(TwoViewReconstruction, PlanarSceneUsesHomography) {
  auto cam = camera();
  std::mt19937 rng(9);
  std::uniform_real_distribution<double> u(-1, 1);
  std::vector<Vec3> X;
  for (int i = 0; i < 300; ++i) { const double x = 2.0 * u(rng), y = 1.3 * u(rng); X.emplace_back(x, y, 4.0 + 0.15 * x - 0.1 * y); }  // tilted plane
  const SE3 gt(SO3::exp(Vec3(0.0, 0.03, 0.0)), Vec3(0.25, -0.02, 0.0));
  const auto s = project(X, gt, *cam, rng, 0.2, 0.0);
  auto r = reconstructTwoView(s.matches, *cam, TwoViewOptions{});
  ASSERT_TRUE(r.has_value());
  EXPECT_TRUE(r->used_homography);
  EXPECT_GT(r->r_h, 0.45);
  expectPose(r->T_c2c1, gt, 0.5, 0.995);
  EXPECT_GE(r->good, static_cast<int>(0.8 * s.matches.size()));
}

TEST(TwoViewReconstruction, RejectsPureRotation) {
  auto cam = camera();
  std::mt19937 rng(2);
  std::uniform_real_distribution<double> u(-1, 1);
  std::vector<Vec3> X;
  for (int i = 0; i < 300; ++i) X.emplace_back(2.0 * u(rng), 1.3 * u(rng), 4.0 + 2.0 * u(rng));
  const SE3 gt(SO3::exp(Vec3(0.0, 0.05, 0.0)), Vec3::Zero());  // no parallax at all
  const auto s = project(X, gt, *cam, rng, 0.3, 0.0);
  EXPECT_FALSE(reconstructTwoView(s.matches, *cam, TwoViewOptions{}).has_value());
}

TEST(TwoViewReconstruction, TooFewMatches) {
  auto cam = camera();
  std::vector<TwoViewMatch> m(5, TwoViewMatch{TrackId{1}, Vec2(10, 10), Vec2(12, 10)});
  EXPECT_FALSE(reconstructTwoView(m, *cam, TwoViewOptions{}).has_value());
}
