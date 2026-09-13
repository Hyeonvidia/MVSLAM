#include "pipelines/pipeline_factory.hpp"

#include "backend/pose_optimizer.hpp"
#include "core/log.hpp"
#include "frontend/depth_image_source.hpp"
#include "frontend/depth_initializer.hpp"
#include "frontend/keyframe_policy.hpp"
#include "frontend/pose_predictor.hpp"
#include "mvslam/api/errors.hpp"
#ifdef MVSLAM_HAVE_OPENCV
#include "frontend/opencv/essential_filter.hpp"
#include "frontend/opencv/orb_descriptor.hpp"
#include "frontend/opencv/stereo_matcher.hpp"
#include "frontend/opencv/grid_detector.hpp"
#include "frontend/opencv/klt_tracker.hpp"
#include "frontend/opencv/mono_initializer.hpp"
#include "frontend/opencv/pnp_solver.hpp"
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
  FrontendConfig fcfg = cfg.frontend;
  if (cfg.execution == ExecutionMode::Offline) fcfg.pose_max_time_ms = 0;  // wall-clock limits break determinism (ADR-0007)
  frontend::FrontendComponents fc;
#ifdef MVSLAM_HAVE_OPENCV
  fc.preprocessor = std::make_unique<frontend::cv_impl::OpenCvPreprocessor>(cfg.frontend);
  fc.tracker = std::make_unique<frontend::cv_impl::KltTracker>(cfg.frontend, *p->rig);
  fc.detector = std::make_unique<frontend::cv_impl::GridShiTomasiDetector>(cfg.frontend);
  fc.outlier_filter = std::make_unique<frontend::cv_impl::EssentialRansacFilter>(cfg.frontend, *p->rig);
  fc.robust_pose = std::make_unique<frontend::cv_impl::P3pRansacSolver>(cfg.frontend, *p->rig);
  p->frontend_available = true;
#else
  MVSLAM_WARN("built without OpenCV: frontend image processing is unavailable (backend/map only)");
#endif
  fc.predictor = std::make_unique<frontend::ConstantVelocityPredictor>();  // ImuPredictor: P2
  fc.pose_optimizer = std::make_unique<backend::MotionOnlyBa>(*p->rig, p->solver, fcfg, cfg.backend);
  fc.keyframe_policy = makeKeyframePolicy(cfg);
  if (!fc.detector || !fc.tracker) {
    fc.initializer = std::make_unique<frontend::NullInitializer>();
  } else if (cfg.sensor == SensorMode::Mono) {
#ifdef MVSLAM_HAVE_OPENCV
    fc.initializer = std::make_unique<frontend::cv_impl::MonoTwoViewInitializer>(cfg.frontend, *fc.detector, *fc.tracker, *p->rig, *p->maps);
#endif
  } else {
    fc.initializer = std::make_unique<frontend::DepthInitializer>(cfg.frontend, cfg.sensor, *fc.detector, *p->rig, *p->maps);
  }
  p->frontend = std::make_unique<frontend::VisualFrontend>(fcfg, cfg.sensor, inertial, std::move(fc), *p->rig, *p->maps);

  p->mapper_logic = std::make_unique<backend::Mapper>(cfg, *p->rig, *p->maps);
  if (cfg.sensor == SensorMode::RgbD) p->mapper_logic->setDepthSource(std::make_unique<frontend::DepthImageSource>(cfg.frontend));
#ifdef MVSLAM_HAVE_OPENCV
  if (cfg.sensor == SensorMode::Stereo) p->mapper_logic->setDepthSource(std::make_unique<frontend::cv_impl::KltStereoMatcher>(cfg.frontend, *p->rig));
  p->mapper_logic->setDescriptorExtractor(std::make_unique<frontend::cv_impl::OrbDescriptorExtractor>());
#endif
  p->lba = std::make_unique<backend::LocalBundleAdjuster>(cfg, *p->rig, p->solver);
  p->culler = std::make_unique<backend::KeyframeCuller>(cfg);

  const auto policy = cfg.execution == ExecutionMode::Offline ? core::ExecutionPolicy::Sync : core::ExecutionPolicy::Async;
  Pipeline* raw = p.get();
  p->optimizer = std::make_unique<core::Worker<OptimizerJob>>(
      "optimizer", policy,
      [raw](OptimizerJob job, std::stop_token st) {
        auto map = raw->maps->active();
        const auto newer = [raw, kf = job.keyframe.value] { return raw->latest_keyframe_submitted.load(std::memory_order_relaxed) != kf; };
        const auto rep = raw->lba->run(*map, job.keyframe, st, newer);
        if (rep.applied && raw->config.backend.kf_culling) {
          const auto removed = map->withWriteLock([&](map::Map& m) { return raw->culler->cull(m, job.keyframe); });
          if (!removed.empty() && raw->maps->events().hasSubscribers())
            for (auto k : removed) raw->maps->events().publish(KeyframeRemovedEvent{map->id().value, k.value});
        }
      },
      [](const OptimizerJob& dropped, OptimizerJob& incoming) {
        incoming.joined.push_back(dropped.keyframe);
        incoming.joined.insert(incoming.joined.end(), dropped.joined.begin(), dropped.joined.end());
      });
  p->mapper = std::make_unique<core::Worker<MapperJob>>(
      "mapper", policy,
      [raw](MapperJob job, std::stop_token st) {
        const auto kf = raw->mapper_logic->process(std::move(job.request), st);
        if (!kf.valid()) return;
        raw->latest_keyframe_submitted.store(kf.value, std::memory_order_relaxed);
        raw->optimizer->submit(OptimizerJob{kf, {}});
        if (raw->loop) raw->loop->submit(LoopJob{kf});
      },
      [](const MapperJob& dropped, MapperJob& incoming) { incoming.dropped.push_back(dropped.request.kf_id); });
  if (cfg.loop.enable || cfg.map.multi_map) {
    p->loop = std::make_unique<core::Worker<LoopJob>>("loop", policy, [raw](LoopJob job, std::stop_token st) {
      (void)job; (void)st; (void)raw;  // place recognition / correction: P3
    });
  }
  MVSLAM_INFO("pipeline: sensor={} inertial={} execution={} solver={} cameras={} frontend={}",
              static_cast<int>(cfg.sensor), static_cast<int>(cfg.inertial), static_cast<int>(cfg.execution),
              p->solver->name(), p->rig->numCameras(), p->frontend_available);
  return p;
}

}  // namespace mvslam::pipelines
