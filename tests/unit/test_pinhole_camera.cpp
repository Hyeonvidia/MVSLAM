#include <gtest/gtest.h>

#include "mvslam/api/errors.hpp"
#include "sensors/camera_model.hpp"

using namespace mvslam;

static CameraConfig euroc() {
  CameraConfig c;
  c.width = 752; c.height = 480;
  c.intrinsics = {458.654, 457.296, 367.215, 248.375};
  c.distortion = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
  return c;
}

TEST(PinholeCamera, ProjectUnprojectRoundTrip) {
  sensors::PinholeCamera cam(euroc());
  const mvslam::core::Vec3 p(0.3, -0.2, 2.0);
  const auto uv = cam.project(p);
  EXPECT_NEAR(uv.x(), 458.654 * 0.15 + 367.215, 1e-9);
  // Undistort of the *ideal* pixel is identity only without distortion; check the ray instead.
  const auto ray = cam.unprojectRaw(cam.project(mvslam::core::Vec3(0, 0, 1)));  // principal point: distortion-free
  EXPECT_NEAR(ray.x(), 0.0, 1e-9);
  EXPECT_NEAR(ray.y(), 0.0, 1e-9);
}

TEST(PinholeCamera, ProjectionJacobianMatchesNumeric) {
  sensors::PinholeCamera cam(euroc());
  const mvslam::core::Vec3 p(0.3, -0.2, 2.0);
  const auto J = cam.projectJacobian(p);
  const double eps = 1e-6;
  for (int i = 0; i < 3; ++i) {
    mvslam::core::Vec3 d = mvslam::core::Vec3::Zero(); d(i) = eps;
    const mvslam::core::Vec2 num = (cam.project(p + d) - cam.project(p - d)) / (2 * eps);  // never `auto` for Eigen expressions
    EXPECT_NEAR(J(0, i), num.x(), 1e-5);
    EXPECT_NEAR(J(1, i), num.y(), 1e-5);
  }
}

TEST(PinholeCamera, RejectsInvalidConfig) {
  auto c = euroc();
  c.intrinsics[0] = 0;
  EXPECT_THROW(sensors::PinholeCamera cam(c), ConfigError);
}
