#include <gtest/gtest.h>

#include "mvslam/api/errors.hpp"
#include "sensors/camera_model.hpp"

using namespace mvslam::core;

namespace {
mvslam::CameraConfig tumVi() {  // TUM-VI cam0 (512x512 fisheye)
  mvslam::CameraConfig c;
  c.model = mvslam::CameraModelType::KannalaBrandt8;
  c.width = 512; c.height = 512;
  c.intrinsics = {190.97847715128717, 190.9733070521226, 254.93170605935475, 256.8974428996504};
  c.distortion = {0.0034823894022493434, 0.0007150348452162257, -0.0020532361418706202, 0.00020293673591811182};
  return c;
}
}  // namespace

TEST(KannalaBrandt8, RawUndistortRoundTripAndRays) {
  auto cam = mvslam::sensors::makeCameraModel(tumVi());
  ASSERT_EQ(cam->type(), mvslam::CameraModelType::KannalaBrandt8);
  const auto& kb = static_cast<const mvslam::sensors::KannalaBrandt8Camera&>(*cam);
  // Principal point maps to the optical axis.
  EXPECT_LT((cam->unprojectRaw(Vec2(kb.cx(), kb.cy())) - Vec3(0, 0, 1)).norm(), 1e-12);
  // raw -> undistorted -> raw inside the pinhole-representable field of view (theta < ~85 deg, i.e.
  // within ~280 px of the principal point for this lens; corners of a 512x512 fisheye lie beyond 90 deg).
  double max_err = 0;
  for (int y = 20; y < 512; y += 40)
    for (int x = 20; x < 512; x += 40) {
      const Vec2 raw(x, y);
      if ((raw - Vec2(kb.cx(), kb.cy())).norm() > 240) continue;
      const Vec2 back = cam->undistortToRaw(cam->undistort(raw));
      max_err = std::max(max_err, (back - raw).norm());
    }
  EXPECT_LT(max_err, 1e-6);
  // A ray at 60 deg off-axis: projectRaw then unprojectRaw returns the same direction.
  const double theta = 60.0 * M_PI / 180.0;
  const Vec3 ray(std::sin(theta) * std::cos(0.7), std::sin(theta) * std::sin(0.7), std::cos(theta));
  const Vec2 px = kb.projectRaw(ray);
  EXPECT_TRUE(cam->inImage(px));
  const Vec3 back = cam->unprojectRaw(px);
  EXPECT_LT((back.normalized() - ray.normalized()).norm(), 1e-9);
  // Undistorted pixels obey the ideal pinhole: project(unprojectUndistorted(u)) == u.
  const Vec2 u(300.0, 200.0);
  EXPECT_LT((cam->project(cam->unprojectUndistorted(u)) - u).norm(), 1e-12);
  // Jacobian of the ideal projection matches finite differences.
  const Vec3 p(0.3, -0.2, 1.5);
  const auto J = cam->projectJacobian(p);
  for (int i = 0; i < 3; ++i) {
    Vec3 d = Vec3::Zero(); d(i) = 1e-6;
    const Vec2 num = (cam->project(p + d) - cam->project(p - d)) / 2e-6;
    EXPECT_LT((num - J.col(i)).norm(), 1e-5) << i;
  }
}

TEST(KannalaBrandt8, RejectsMissingCoefficients) {
  auto c = tumVi();
  c.distortion = {0.1};
  EXPECT_THROW(mvslam::sensors::makeCameraModel(c), mvslam::ConfigError);
}
