#pragma once
// Template Method orchestrating one frame (docs/design/01-architecture.md §3.1). All algorithmic
// steps are injected strategies; this class owns only sequencing and the state machine.
#include <memory>

#include "frontend/interfaces.hpp"
#include "frontend/tracking_state_machine.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::frontend {

struct FrontendComponents {
  std::unique_ptr<ImagePreprocessor> preprocessor;
  std::unique_ptr<IFeatureTracker> tracker;
  std::unique_ptr<IOutlierFilter> outlier_filter;
  std::unique_ptr<IPosePredictor> predictor;
  std::unique_ptr<IPoseOptimizer> pose_optimizer;   // from backend
  std::unique_ptr<IKeyframePolicy> keyframe_policy;
};

struct KeyframeRequest {  // what the frontend hands to the mapper worker
  Frame frame;
  std::shared_ptr<ImageState> image_state;   // pyramids for detection / stereo matching
  std::vector<std::uint8_t> raw_image_copy;  // descriptors are computed on the raw image at KF time
  int width = 0, height = 0, stride = 0;
};

class VisualFrontend {
 public:
  VisualFrontend(FrontendConfig cfg, bool inertial, FrontendComponents c);
  // Returns the result and, when a keyframe was decided, fills `kf_request`.
  TrackResult process(const FrameInput& input, std::optional<KeyframeRequest>& kf_request);
  const Frame& lastFrame() const { return last_; }
  TrackingState state() const { return sm_.state(); }
  void setOptimizerBusy(bool b) { optimizer_busy_ = b; }

 private:
  FrontendConfig cfg_;
  FrontendComponents c_;
  TrackingStateMachine sm_;
  std::unique_ptr<ImageState> last_image_;
  Frame last_;
  bool has_last_ = false;
  int frames_since_kf_ = 0;
  bool optimizer_busy_ = false;
};

}  // namespace mvslam::frontend
