#pragma once
// Template Method orchestrating one frame (docs/design/01-architecture.md §3.1). All algorithmic
// steps are injected strategies; this class owns sequencing, the bootstrap phases and the state
// machine. Runs on the caller thread; reads the map under its read lock only.
#include <memory>

#include "frontend/interfaces.hpp"
#include "frontend/relocalizer.hpp"
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
  std::unique_ptr<IDescriptorExtractor> descriptor;   // LOST-time descriptors (relocalization)
  std::unique_ptr<IFeatureDetector> reloc_detector;    // denser grid for relocalization queries
  std::unique_ptr<Relocalizer> relocalizer;            // null: relocalization disabled
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
  // Stateless single-image localization against every map (stored + active); tracking is untouched.
  std::optional<RelocalizationResult> localizeOnce(const FrameInput& input);
  // Global relocalization for the next `max_attempts` frames (MapService::localize): while armed, the
  // bootstrap and LOST paths search every map instead of the active one and switch maps on success.
  void armGlobalRelocalization(int max_attempts) { reloc_armed_ = max_attempts; reloc_outcome_.reset(); }
  std::optional<Result<Pose>> takeRelocalizationOutcome() { auto o = std::move(reloc_outcome_); reloc_outcome_.reset(); return o; }

 private:
  enum class Phase { Bootstrap, WaitingFor3d, Tracking };
  void refreshTracksFromMap(Frame& f);
  bool estimatePose(Frame& cur, const IFeatureTracker::Stats& st, TrackResult& r);
  void createKeyframe(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, KeyframeRequest& out);
  void restartBootstrap();
  int countOccupiedCells(const Frame& f) const;
  bool tryRelocalize(Frame& cur, const std::shared_ptr<ImageState>& image, const FrameInput& input, bool all_maps);
  void adoptRelocalization(Frame& cur, const RelocalizationResult& rr, Timestamp t);

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
  int reloc_armed_ = 0;
  std::optional<Result<Pose>> reloc_outcome_;
};

}  // namespace mvslam::frontend
