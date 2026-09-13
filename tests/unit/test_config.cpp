#include <gtest/gtest.h>

#include "mvslam/api/config.hpp"
#include "mvslam/api/errors.hpp"

using namespace mvslam;

static const char* kMono = R"(
schema: 1
sensor: mono
inertial: none
execution: offline
rig:
  cameras:
    - {model: pinhole, width: 752, height: 480, intrinsics: [458.654, 457.296, 367.215, 248.375], distortion: [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]}
frontend: {klt_window: 9, klt_levels: 3}
backend: {solver: ceres, lba_max_keyframes: 20}
loop: {enable: false}
)";

TEST(Config, ParsesAndValidates) {
  auto c = Config::fromYamlString(kMono);
  EXPECT_EQ(c.sensor, SensorMode::Mono);
  EXPECT_EQ(c.execution, ExecutionMode::Offline);
  ASSERT_EQ(c.rig.cameras.size(), 1u);
  EXPECT_DOUBLE_EQ(c.rig.cameras[0].intrinsics[0], 458.654);
  EXPECT_EQ(c.backend.lba_max_keyframes, 20);
  EXPECT_FALSE(c.toYaml().empty());
}

TEST(Config, RejectsContradictions) {
  std::string stereo_one_cam = kMono;
  stereo_one_cam.replace(stereo_one_cam.find("sensor: mono"), 12, "sensor: stereo");
  EXPECT_THROW(Config::fromYamlString(stereo_one_cam), ConfigError);

  std::string imu_without_calib = kMono;
  imu_without_calib.replace(imu_without_calib.find("inertial: none"), 14, "inertial: imu");
  EXPECT_THROW(Config::fromYamlString(imu_without_calib), ConfigError);

  std::string offline_threads = kMono;
  offline_threads.replace(offline_threads.find("lba_max_keyframes: 20"), 21, "lba_max_keyframes: 20, solver_threads: 4");
  EXPECT_THROW(Config::fromYamlString(offline_threads), ConfigError);
}

TEST(Config, ParsesSolverBackend) {
  std::string y = kMono;
  y.replace(y.find("solver: ceres"), 13, "solver: g2o");
  auto c = Config::fromYamlString(y);
  EXPECT_EQ(c.backend.solver, SolverBackend::G2o);
}
