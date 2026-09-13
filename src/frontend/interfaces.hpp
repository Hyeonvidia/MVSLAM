#pragma once
// Frontend strategy interfaces (docs/design/02-frontend.md §1). The frontend never includes
// backend headers: IPoseOptimizer is declared here and implemented in backend/, injected by pipelines/.
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "core/ids.hpp"
#include "core/math/se3.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam::frontend {

struct Track {
  core::TrackId id;
  core::Vec2 uv_raw, uv;     // current raw / undistorted pixel
  int cam = 0;
  int age = 0;               // frames since detection
  bool is3d = false;         // has an associated map point
  core::MapPointId map_point;
  core::Vec3 X_w;            // snapshot of the map point position (valid when is3d)
  double fb_err = 0;         // forward-backward KLT error of the last step
  double info = 1.0;         // observation weight (Sigma^-1 = info * I)
};

struct Frame {
  Timestamp timestamp = 0;
  core::SE3 T_wb;
  core::Vec3 v_w = core::Vec3::Zero();
  core::Vec6 bias = core::Vec6::Zero();
  std::vector<Track> tracks;
  core::KeyframeId reference_kf;
  core::SE3 T_kb;            // pose relative to the reference keyframe (auto-corrected by PGO)
  int occupied_cells = 0;
};

// Opaque per-frame image state (pyramids etc.) owned by the tracker implementation.
struct ImageState { virtual ~ImageState() = default; };

class ImagePreprocessor {
 public:
  virtual ~ImagePreprocessor() = default;
  virtual std::unique_ptr<ImageState> process(const ImageView& image) = 0;
};

class IFeatureTracker {
 public:
  virtual ~IFeatureTracker() = default;
  struct Stats { int attempted3d = 0, survived3d = 0, attempted2d = 0, survived2d = 0; };
  // Propagates prev.tracks into cur.tracks using the predicted pose for 3D tracks.
  virtual Stats track(const ImageState& prev, const ImageState& cur, const Frame& prev_frame, Frame& cur_frame,
                      const core::SE3& predicted_T_wb) = 0;
};

class IFeatureDetector {  // keyframe-time only
 public:
  virtual ~IFeatureDetector() = default;
  virtual std::vector<core::Vec2> detect(const ImageState& image, const std::vector<core::Vec2>& existing, int max_new) = 0;
};

class IDescriptorExtractor {  // keyframe-time and LOST-time only
 public:
  virtual ~IDescriptorExtractor() = default;
  virtual std::vector<std::uint8_t> describe(const ImageView& raw, const std::vector<core::Vec2>& uv_raw) = 0;
  virtual int stride() const = 0;  // bytes per descriptor
};

class IOutlierFilter {
 public:
  virtual ~IOutlierFilter() = default;
  virtual int filter(const Frame& reference, Frame& cur) = 0;  // returns #removed
};

class IPosePredictor {
 public:
  virtual ~IPosePredictor() = default;
  virtual core::SE3 predict(const Frame& last, Timestamp t_next) = 0;
  virtual void update(const Frame& last, const Frame& cur) = 0;
};

struct PoseOptimizationResult { bool ok = false; int inliers = 0; double chi2 = 0; };

class IPoseOptimizer {  // implemented in backend/, see backend/pose_optimizer.hpp
 public:
  virtual ~IPoseOptimizer() = default;
  virtual PoseOptimizationResult optimize(Frame& frame, double max_time_ms) = 0;
};

class IKeyframePolicy {
 public:
  virtual ~IKeyframePolicy() = default;
  struct Context {
    const Frame* cur = nullptr;
    const Frame* last_kf = nullptr;
    int frames_since_kf = 0;
    double seconds_since_kf = 0;
    double median_parallax_px = 0;
    int max_tracks = 0;
    bool optimizer_busy = false;
    bool imu_initialized = false;
    int tracked_close = 0, new_close = 0;
  };
  virtual bool needKeyframe(const Context& ctx) const = 0;
};

}  // namespace mvslam::frontend
