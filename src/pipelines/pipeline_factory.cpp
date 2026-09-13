#include "pipelines/pipeline_factory.hpp"

#include "backend/pose_optimizer.hpp"
#include "core/log.hpp"
#include "frontend/keyframe_policy.hpp"
#include "mvslam/api/errors.hpp"
#ifdef MVSLAM_HAVE_OPENCV
#include "frontend/opencv/klt_tracker.hpp"
#include "frontend/opencv/preprocessor.hpp"
#endif

namespace mvslam::pipelines {

namespace {

std::unique_ptr<frontend::IKeyframePolicy> makeKeyframePolicy(const Config& cfg) {
  std::unique_ptr<frontend::IKeyframePolicy> p = std::make_unique<frontend::ParallaxTrackRatioPolicy>(cfg.frontend);
  if (cfg.sensor != SensorMode::Mono) p = std::make_unique<frontend::ClosePointRulePolicy>(std::move(p), cfg.frontend);
  if (cfg.inertial == InertialMode::Imu) p = std::make_unique<frontend::InertialTimeRulePolicy>(std::move(p), cfg.frontend);
  return p;
}

}  // namespace

std::unique_ptr<Pipeline> PipelineFactory::build(const Config& cfg) {
  cfg.validate();
  auto p = std::make_unique<Pipeline>();
  p->config = cfg;
  p->rig = std::make_unique<sensors::CameraRig>(cfg.rig);
  p->maps = std::make_unique<map::MapStore>();
  p->maps->setPlaceRecognizer(std::make_unique<map::NullPlaceRecognizer>());  // DBoW/incremental adapters: P3
  p->solver = backend::graph::makeSolver(cfg.backend.solver);

  const bool inertial = cfg.inertial == InertialMode::Imu;
  frontend::FrontendComponents fc;
#ifdef MVSLAM_HAVE_OPENCV
  fc.preprocessor = std::make_unique<frontend::cv_impl::OpenCvPreprocessor>(cfg.frontend);
  fc.tracker = std::make_unique<frontend::cv_impl::KltTracker>(cfg.frontend, *p->rig);
  p->frontend_available = true;
#else
  MVSLAM_WARN("built without OpenCV: frontend image processing is unavailable (backend/map only)");
#endif
  fc.pose_optimizer = std::make_unique<backend::MotionOnlyBa>(*p->rig, p->solver, cfg.frontend, cfg.backend);
  fc.keyframe_policy = makeKeyframePolicy(cfg);
  // Mode-specific strategies still to be wired (P0-P2): initializer, outlier filter, IMU predictor,
  // depth source (KLT stereo / depth image), pose-inertial optimizer. See docs/design/09-roadmap.md.
  p->frontend = std::make_unique<frontend::VisualFrontend>(cfg.frontend, inertial, std::move(fc));

  const auto policy = cfg.execution == ExecutionMode::Offline ? core::ExecutionPolicy::Sync : core::ExecutionPolicy::Async;
  Pipeline* raw = p.get();
  p->mapper = std::make_unique<core::Worker<MapperJob>>(
      "mapper", policy,
      [raw](MapperJob job, std::stop_token) {
        // P0: detect/describe at keyframe, depth source, triangulation, local-map matching, then
        // hand the keyframe id to the optimizer. Skeleton: forward only.
        (void)job;
        (void)raw;
      },
      [](const MapperJob& dropped, MapperJob& incoming) {
        incoming.merged_from_dropped.push_back(dropped.request.frame.reference_kf);
      });
  p->optimizer = std::make_unique<core::Worker<OptimizerJob>>(
      "optimizer", policy,
      [raw](OptimizerJob job, std::stop_token st) { (void)job; (void)st; (void)raw; },
      [](const OptimizerJob& dropped, OptimizerJob& incoming) {
        incoming.joined.push_back(dropped.keyframe);
        incoming.joined.insert(incoming.joined.end(), dropped.joined.begin(), dropped.joined.end());
      });
  if (cfg.loop.enable || cfg.map.multi_map) {
    p->loop = std::make_unique<core::Worker<LoopJob>>("loop", policy, [raw](LoopJob job, std::stop_token st) {
      (void)job; (void)st; (void)raw;
    });
  }
  MVSLAM_INFO("pipeline: sensor={} inertial={} execution={} solver={} cameras={} frontend={}",
              static_cast<int>(cfg.sensor), static_cast<int>(cfg.inertial), static_cast<int>(cfg.execution),
              p->solver->name(), p->rig->numCameras(), p->frontend_available);
  return p;
}

}  // namespace mvslam::pipelines
