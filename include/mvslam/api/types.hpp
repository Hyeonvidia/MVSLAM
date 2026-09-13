#pragma once
// Public value types. Rules (docs/design/05-api-config.md §1):
//  * plain structs / standard containers only — no Eigen, OpenCV or Sophus here;
//  * poses are body-to-world (T_wb); images are non-owning views valid only during the call.
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace mvslam {

using Timestamp = std::int64_t;  // nanoseconds

struct Quat { double w = 1, x = 0, y = 0, z = 0; };
struct Vec3 { double x = 0, y = 0, z = 0; };
struct Pose { Quat q; Vec3 t; };                                   // T_wb
struct PoseStamped { Timestamp timestamp = 0; Pose T_wb; };
struct PoseState { Pose T_wb; Vec3 v_w; Vec3 bias_gyro; Vec3 bias_acc; };

enum class PixelFormat : std::uint8_t { Gray8, Rgb8, Bgr8, Depth16, DepthF32 };

struct ImageView {
  const std::uint8_t* data = nullptr;
  int width = 0, height = 0;
  int stride = 0;  // bytes per row
  PixelFormat format = PixelFormat::Gray8;
};

struct FrameInput {
  Timestamp timestamp = 0;
  std::vector<ImageView> images;      // one per rig camera, in rig order (mono: 1, stereo: 2)
  std::optional<ImageView> depth;     // RgbD only, registered to camera 0
};

struct ImuSample { Timestamp timestamp = 0; Vec3 gyro; Vec3 acc; };  // rad/s, m/s^2, body frame

enum class TrackingState : std::uint8_t { NotInitialized, Ok, RecentlyLost, Lost };

struct TrackStats {
  int tracks = 0, tracks3d = 0, inliers = 0;
  bool keyframe_inserted = false;
  double frontend_ms = 0;
};

struct TrackResult {
  Timestamp timestamp = 0;
  TrackingState state = TrackingState::NotInitialized;
  std::optional<PoseState> pose;            // absent while not initialized / lost
  std::optional<std::uint64_t> keyframe_id;  // set when this frame became a keyframe
  TrackStats stats;
};

// Error policy class 3 (async operations): a value or a message, never an exception across threads.
template <typename T>
struct Result {
  bool ok = false;
  std::string error;
  std::optional<T> value;
  static Result success(T v) { return Result{true, {}, std::move(v)}; }
  static Result failure(std::string msg) { return Result{false, std::move(msg), std::nullopt}; }
};
template <>
struct Result<void> {
  bool ok = false;
  std::string error;
  static Result success() { return Result{true, {}}; }
  static Result failure(std::string msg) { return Result{false, std::move(msg)}; }
};

}  // namespace mvslam
