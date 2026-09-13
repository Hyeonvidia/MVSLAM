#pragma once
// Template Method orchestrating one frame (docs/design/01-architecture.md §3.1). All algorithmic
// steps are injected strategies; this class owns sequencing, the bootstrap phases and the state
// machine. Runs on the caller thread; reads the map under its read lock only.
#include <memory>

#include "frontend/interfaces.hpp"
#include "frontend/tracking_state_machine.hpp"
#include "map/map_store.hpp"
#include "mvslam/api/config.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend {

struct FrontendComponents {
  std::unique_ptr<ImagePreprocessor> preprocessor;
  std::unique_ptr<IFeatureTracker> tracker;
  std::unique_ptr<IFeatureDetector> detector;        // keyframe-time detection (also used by initializers)
  std::unique_ptr<IOutlierFilter> outlier_filter;
  std::unique_ptr<IPosePredictor> predictor;
  std::unique_ptr<IRobustPoseSolver> robust_pose;    // P3P fallback
  std::unique_ptr<IPoseOptimizer> pose_optimizer;    // from backend
  std::unique_ptr<IKeyframePolicy> keyframe_policy;
  std::unique_ptr<IInitializer> initializer;
};

class VisualFrontend {
 public:
  VisualFrontend(FrontendConfig cfg, SensorMode sensor, bool inertial, FrontendComponents c,
                 const sensors::CameraRig& rig, map::MapStore& maps);
  // Returns the result and, when a keyframe was decided, fills `kf_request`.
  TrackResult process(const FrameInput& input, std::optional<KeyframeRequest>& kf_request);
  const Frame& lastFrame() const { return last_; }
  TrackingState state() const { return sm_.state(); }
  void setOptimizerBusy(bool b) { optimizer_busy_ = b; }

 private:
  enum class Phase { Bootstrap, WaitingFor3d, Tracking };
  void refreshTracksFromMap(Frame& f);
  bool estimatePose(Frame& cur, const IFeatureTracker::Stats& st, TrackResult& r);
  void createKeyframe(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out);
  void restartBootstrap();
  int countOccupiedCells(const Frame& f) const;

  FrontendConfig cfg_;
  SensorMode sensor_;
  FrontendComponents c_;
  const sensors::CameraRig& rig_;
  map::MapStore& maps_;
  TrackingStateMachine sm_;
  Phase phase_ = Phase::Bootstrap;
  std::shared_ptr<ImageState> last_image_;
  Frame last_, last_kf_frame_;
  core::KeyframeId last_kf_id_;
  bool has_last_ = false;
  bool p3p_required_ = false;
  int frames_since_kf_ = 0, waiting_frames_ = 0;
  std::uint64_t frame_counter_ = 0, last_epoch_ = 0;
  int max_tracks_ = 0;
  bool optimizer_busy_ = false;
};

}  // namespace mvslam::frontend
