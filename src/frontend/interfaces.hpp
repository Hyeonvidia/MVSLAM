#pragma once
// Frontend strategy interfaces (docs/design/02-frontend.md §1). The frontend never includes
// backend headers: IPoseOptimizer is declared here and implemented in backend/, injected by pipelines/.
// Id convention: a track keeps its TrackId for life; once triangulated, the map point has the same
// numeric id (MapPointId{track.value}) — no separate association table (OV2SLAM lmid convention).
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "core/ids.hpp"
#include "core/image_buffer.hpp"
#include "core/math/se3.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam::frontend {

inline core::MapPointId toMapPointId(core::TrackId t) { return core::MapPointId{t.value}; }

struct Track {
  core::TrackId id;
  core::Vec2 uv_raw, uv;     // current raw / undistorted pixel (camera 0)
  int cam = 0;
  int age = 0;               // frames since detection
  bool is3d = false;         // map point with this id exists (position snapshot in X_w)
  core::Vec3 X_w = core::Vec3::Zero();
  double info = 1.0;         // observation weight (Sigma^-1 = info * I)
};

struct Frame {
  std::uint64_t id = 0;      // frame counter
  Timestamp timestamp = 0;
  core::SE3 T_wb;
  core::Vec3 v_w = core::Vec3::Zero();
  core::Vec6 bias = core::Vec6::Zero();
  std::vector<Track> tracks;
  core::KeyframeId reference_kf;   // keyframe this frame's pose is expressed against
  core::SE3 T_kb;                  // T_wb = T_wk(reference) * T_kb  (auto-corrected when the KF moves)
  int occupied_cells = 0;
  int num3d() const { int n = 0; for (const auto& t : tracks) n += t.is3d ? 1 : 0; return n; }
};

// Opaque per-frame image state (pyramids etc.) owned by the tracker implementation.
struct ImageState { virtual ~ImageState() = default; };

// What the frontend hands to the mapper worker when a keyframe is created.
struct KeyframeRequest {
  core::KeyframeId kf_id;           // allocated by the frontend so following frames can reference it
  Frame frame;                      // tracks incl. newly detected 2-D tracks
  std::shared_ptr<ImageState> image_state;   // left pyramid (stereo matching prior, detection)
  core::ImageBuffer left_raw;       // descriptors are computed on the raw image
  core::ImageBuffer right_raw;      // stereo only
  core::ImageBuffer depth;          // rgbd only
  bool initial = false;             // first keyframe of a map (stereo/rgbd bootstrap)
  // Mono bootstrap: a complete two-view initial map (poses + points), inserted verbatim by the mapper.
  struct InitialMap {
    core::KeyframeId first_kf;
    Frame first_frame;
    core::ImageBuffer first_left_raw;
    std::vector<std::pair<core::TrackId, core::Vec3>> points;  // triangulated tracks
  };
  std::optional<InitialMap> initial_map;
};

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
  virtual int cellCount(int width, int height) const = 0;  // max tracks budget (one per cell)
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
  virtual void reset() = 0;
};

// Robust pose from 2D-3D correspondences (P3P-RANSAC); used when KLT collapsed or PnP failed.
class IRobustPoseSolver {
 public:
  virtual ~IRobustPoseSolver() = default;
  virtual bool solve(const Frame& frame, core::SE3& T_wb, std::vector<int>& outlier_indices) = 0;
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

// Keyframe-time depth: stereo matching (KLT left->right) or a registered depth image.
struct DepthMeasurement {
  double depth = -1;            // metric depth along the camera-0 ray of the (undistorted) keypoint, <0 = none
  bool has_right = false;       // stereo: matched undistorted pixel in camera 1 (extra BA observation)
  core::Vec2 uv_right = core::Vec2::Zero();
};
class IDepthSource {
 public:
  virtual ~IDepthSource() = default;
  // One entry per track of req.frame.tracks (same order).
  virtual std::vector<DepthMeasurement> measure(const KeyframeRequest& req) = 0;
};

// Bootstraps the map. Stereo/RGB-D: first frame becomes a keyframe (depth from the mapper).
// Mono: two-view reconstruction after enough parallax. Returns true when `out` was filled.
class IInitializer {
 public:
  virtual ~IInitializer() = default;
  virtual bool tryInitialize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input,
                             KeyframeRequest& out) = 0;
  virtual void reset() = 0;
};

}  // namespace mvslam::frontend
