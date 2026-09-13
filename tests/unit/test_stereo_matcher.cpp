#include <gtest/gtest.h>

#include <opencv2/imgproc.hpp>

#include <random>

#include "frontend/opencv/preprocessor.hpp"
#include "frontend/opencv/stereo_matcher.hpp"

using namespace mvslam::core;

TEST(KltStereoMatcher, RecoversDepthFromSyntheticDisparity) {
  const int W = 640, H = 480, disparity = 16;
  const double fx = 500.0, baseline = 0.1;
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = W; c0.height = H; c0.intrinsics = {fx, fx, 320.0, 240.0};
  mvslam::CameraConfig c1 = c0; c1.T_bc = {1, 0, 0, baseline, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  rc.cameras = {c0, c1};
  mvslam::sensors::CameraRig rig(rc);
  mvslam::FrontendConfig fc;
  fc.clahe = false;

  // Left: smoothed random texture; right: the left image shifted so that u_right = u_left - disparity.
  cv::Mat noise(H, W + disparity, CV_8UC1), texture;
  cv::RNG cvrng(42);
  cvrng.fill(noise, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(noise, texture, cv::Size(0, 0), 1.2);
  cv::normalize(texture, texture, 0, 255, cv::NORM_MINMAX);
  cv::Mat left = texture(cv::Rect(0, 0, W, H)).clone();
  cv::Mat right = texture(cv::Rect(disparity, 0, W, H)).clone();

  mvslam::frontend::cv_impl::OpenCvPreprocessor pp(fc);
  mvslam::ImageView lv{left.data, W, H, static_cast<int>(left.step), mvslam::PixelFormat::Gray8};
  mvslam::ImageView rv{right.data, W, H, static_cast<int>(right.step), mvslam::PixelFormat::Gray8};
  mvslam::frontend::KeyframeRequest req;
  req.image_state = std::shared_ptr<mvslam::frontend::ImageState>(pp.process(lv));
  req.left_raw = ImageBuffer::copy(lv);
  req.right_raw = ImageBuffer::copy(rv);
  for (int y = 40; y < H - 40; y += 40) {
    for (int x = 40 + disparity; x < W - 40; x += 40) {
      mvslam::frontend::Track t;
      t.id = TrackId{static_cast<std::uint64_t>(req.frame.tracks.size() + 1)};
      t.uv = t.uv_raw = Vec2(x, y);
      req.frame.tracks.push_back(t);
    }
  }
  mvslam::frontend::cv_impl::KltStereoMatcher matcher(fc, rig);
  const auto depth = matcher.measure(req);
  ASSERT_EQ(depth.size(), req.frame.tracks.size());
  const double expected = fx * baseline / disparity;  // 3.125 m
  int accepted = 0;
  for (std::size_t i = 0; i < depth.size(); ++i) {
    if (depth[i].depth < 0) continue;
    ++accepted;
    EXPECT_TRUE(depth[i].has_right);
    EXPECT_NEAR(depth[i].depth, expected, 0.03) << "track " << i;
    EXPECT_NEAR(depth[i].uv_right.x(), req.frame.tracks[i].uv.x() - disparity, 0.3) << "track " << i;
    EXPECT_NEAR(depth[i].uv_right.y(), req.frame.tracks[i].uv.y(), 0.3) << "track " << i;
  }
  EXPECT_GE(accepted, static_cast<int>(0.9 * static_cast<double>(depth.size())));
}

TEST(KltStereoMatcher, NoRightImageYieldsNoDepth) {
  mvslam::RigConfig rc;
  mvslam::CameraConfig c0; c0.width = 64; c0.height = 48; c0.intrinsics = {50, 50, 32, 24};
  mvslam::CameraConfig c1 = c0; c1.T_bc = {1, 0, 0, 0.1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  rc.cameras = {c0, c1};
  mvslam::sensors::CameraRig rig(rc);
  mvslam::FrontendConfig fc;
  mvslam::frontend::cv_impl::KltStereoMatcher matcher(fc, rig);
  mvslam::frontend::KeyframeRequest req;
  req.frame.tracks.resize(3);
  const auto depth = matcher.measure(req);
  ASSERT_EQ(depth.size(), 3u);
  for (const auto& d : depth) EXPECT_LT(d.depth, 0);
}
