#include "pipelines/pipeline_factory.hpp"

#include "backend/pose_optimizer.hpp"
#include "core/math/se3.hpp"
#include "map/hbst_place_recognizer.hpp"
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
  RigConfig rig_cfg = cfg.rig;
  if (cfg.sensor == SensorMode::RgbD && rig_cfg.cameras.size() == 1) {
    // Virtual right camera at rgbd_virtual_baseline_m along the camera x-axis: depth measurements are
    // expressed as a right-view observation u_r = u - fx*b/z and enter BA as a second reprojection
    // residual (ORB-SLAM2 RGB-D). The public config keeps a single camera; only the internal rig has two.
    CameraConfig right = rig_cfg.cameras[0];
    const core::SE3 T_bc0 = core::toSE3(right.T_bc);
    const core::SE3 T_bc1 = T_bc0 * core::SE3(core::SO3(), core::Vec3(cfg.frontend.rgbd_virtual_baseline_m, 0, 0));
    const core::Mat4 M = T_bc1.matrix();
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) right.T_bc[static_cast<std::size_t>(r * 4 + c)] = M(r, c);
    rig_cfg.cameras.push_back(right);
    rig_cfg.stereo_baseline_m = cfg.frontend.rgbd_virtual_baseline_m;
  }
  p->rig = std::make_unique<sensors::CameraRig>(rig_cfg);
  p->maps = std::make_unique<map::MapStore>();
  switch (cfg.loop.recognizer) {  // relocalization (P1) and loop candidates (P3) share the recognizer
    case PlaceRecognizerType::Incremental: p->maps->setPlaceRecognizer(std::make_unique<map::HbstPlaceRecognizer>()); break;
    case PlaceRecognizerType::DBoW: throw ConfigError("loop.recognizer=dbow: adapter not built yet (use incremental)");
    case PlaceRecognizerType::None: p->maps->setPlaceRecognizer(std::make_unique<map::NullPlaceRecognizer>()); break;
  }
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
#ifdef MVSLAM_HAVE_OPENCV
  fc.descriptor = std::make_unique<frontend::cv_impl::OrbDescriptorExtractor>();
  if (cfg.loop.recognizer != PlaceRecognizerType::None && fc.detector && fc.robust_pose) {
    FrontendConfig rcfg = fcfg;
    rcfg.detector_cell_px = fcfg.reloc_detector_cell_px;
    rcfg.ransac_iters = fcfg.reloc_ransac_iters;
    fc.reloc_detector = std::make_unique<frontend::cv_impl::GridShiTomasiDetector>(rcfg);
    fc.reloc_p3p = std::make_unique<frontend::cv_impl::P3pRansacSolver>(rcfg, *p->rig);
    fc.relocalizer = std::make_unique<frontend::Relocalizer>(fcfg, *p->rig, *p->maps, *fc.reloc_detector, *fc.descriptor, *fc.reloc_p3p, *fc.pose_optimizer);
  }
#endif
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
  if (cfg.sensor == SensorMode::RgbD) p->mapper_logic->setDepthSource(std::make_unique<frontend::DepthImageSource>(cfg.frontend, p->rig->camera(0).fx()));
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
        if (rep.applied && raw->maps->events().hasSubscribers()) raw->maps->events().publish(MapPointsUpdatedEvent{map->id().value, map->changeEpoch()});
        if (rep.applied && raw->config.backend.kf_culling) {
          const auto removed = map->withWriteLock([&](map::Map& m) { return raw->culler->cull(m, job.keyframe); });
          for (auto k : removed) if (auto* pr = raw->maps->placeRecognizer()) pr->erase(k);
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
