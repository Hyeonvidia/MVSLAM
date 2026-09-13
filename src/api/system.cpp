#include "mvslam/api/system.hpp"

#include <algorithm>
#include <mutex>

#include "api/map_service_impl.hpp"
#include "core/log.hpp"
#include "core/profiler.hpp"
#include "mvslam/api/errors.hpp"
#include "mvslam/api/version.hpp"
#include "pipelines/pipeline_factory.hpp"

namespace mvslam {

struct System::Impl {
  explicit Impl(Config cfg)
      : pipeline(pipelines::PipelineFactory::build(cfg)),
        map_service(*pipeline->maps, [this](std::function<void(Result<Pose>)> cb) {
          if (!pipeline->frontend) { cb(Result<Pose>::failure("localize: no frontend")); return; }
          localize_cb = std::move(cb);
          pipeline->frontend->armGlobalRelocalization(pipeline->config.frontend.reloc_max_attempts);
        }) {
    // IMU initialization rescales the map: keyframe-relative frame poses must follow (mono scale).
    imu_events = pipeline->maps->events().subscribe([this](const MapEvent& e) {
      if (const auto* ie = std::get_if<ImuInitializedEvent>(&e)) {
        std::lock_guard<std::mutex> lk(frames_mu);
        if (std::abs(ie->scale - 1.0) < 1e-9) return;
        for (auto& f : frames) if (f.map == ie->map_id) f.T_kb = core::SE3(f.T_kb.so3(), ie->scale * f.T_kb.translation());
      }
    });
  }
  std::unique_ptr<pipelines::Pipeline> pipeline;
  api::MapServiceImpl map_service;
  std::vector<ImuSample> imu_buffer;
  Timestamp last_imu = 0, last_frame = 0;
  // Frame poses are kept relative to their reference keyframe so backend corrections propagate.
  struct FrameRecord { Timestamp t; core::KeyframeId kf; core::SE3 T_kb; core::SE3 T_wb_at_track; std::uint64_t map = 0; };
  std::vector<FrameRecord> frames;
  std::mutex frames_mu;
  Subscription imu_events;
  std::function<void(Result<Pose>)> localize_cb;
};

System::System(Config cfg) {
  cfg.validate();
  core::initLogging(cfg.logging.level, cfg.logging.to_stderr, cfg.logging.file);
  impl_ = std::make_unique<Impl>(std::move(cfg));
  MVSLAM_INFO("mvslam {} started", kVersion);
}
System::~System() = default;
System::System(System&&) noexcept = default;
System& System::operator=(System&&) noexcept = default;

void System::pushImu(const ImuSample& s) {
  if (s.timestamp < impl_->last_imu) throw InitError("pushImu: timestamps must be non-decreasing");
  impl_->last_imu = s.timestamp;
  impl_->imu_buffer.push_back(s);
}

TrackResult System::track(const FrameInput& input) {
  if (input.timestamp < impl_->last_frame) throw InitError("track: frame timestamps must be non-decreasing");
  impl_->last_frame = input.timestamp;
  auto& p = *impl_->pipeline;
  if (!p.frontend_available) {
    TrackResult r;
    r.timestamp = input.timestamp;
    r.state = TrackingState::NotInitialized;
    return r;
  }
  std::optional<frontend::KeyframeRequest> kf;
  p.frontend->setOptimizerBusy(p.optimizer && (p.optimizer->busy() || p.optimizer->pending()));
  // IMU samples up to this frame (the buffer keeps only later ones); the frontend interpolates at the frame times.
  std::vector<ImuSample> imu;
  {
    auto& buf = impl_->imu_buffer;
    auto it = std::find_if(buf.begin(), buf.end(), [&](const ImuSample& s) { return s.timestamp > input.timestamp; });
    imu.assign(buf.begin(), it);
    buf.erase(buf.begin(), it);
  }
  TrackResult r = p.frontend->process(input, imu, kf);
  if (kf) p.mapper->submit(pipelines::MapperJob{std::move(*kf), {}});
  if (r.pose) {
    const auto& f = p.frontend->lastFrame();
    std::lock_guard<std::mutex> lk(impl_->frames_mu);
    impl_->frames.push_back({r.timestamp, f.reference_kf, f.T_kb, f.T_wb, p.maps->active()->id().value});
  }
  r.stats.frontend_ms = core::Profiler::instance().get("frontend.process").mean_ms;
  if (auto outcome = p.frontend->takeRelocalizationOutcome(); outcome && impl_->localize_cb) {
    auto cb = std::move(impl_->localize_cb);
    impl_->localize_cb = nullptr;
    cb(*outcome);
  }
  return r;
}

Result<Pose> System::localize(const FrameInput& input) {
  auto& p = *impl_->pipeline;
  if (!p.frontend_available) return Result<Pose>::failure("localize: frontend unavailable");
  const auto rr = p.frontend->localizeOnce(input);
  if (!rr) return Result<Pose>::failure("localize: no map matched");
  return Result<Pose>::success(core::fromSE3(rr->T_wb));
}

MapService& System::mapService() { return impl_->map_service; }

std::vector<PoseStamped> System::trajectory() const {
  std::vector<PoseStamped> out;
  out.reserve(impl_->frames.size());
  auto map = impl_->pipeline->maps->active();
  std::lock_guard<std::mutex> lk(impl_->frames_mu);
  map->withReadLock([&](const map::Map& m) {
    for (const auto& f : impl_->frames) {
      const auto T_wk = m.resolvePose(f.kf);  // live keyframe, or culled chain to a live one
      const core::SE3 T = T_wk ? *T_wk * f.T_kb : f.T_wb_at_track;  // reference never inserted (dropped job / other map)
      out.push_back(PoseStamped{f.t, core::fromSE3(T)});
    }
    return 0;
  });
  return out;
}

SystemStats System::stats() const {
  SystemStats s;
  s.frontend_ms_avg = core::Profiler::instance().get("frontend.process").mean_ms;
  s.mapper_ms_avg = core::Profiler::instance().get("mapper.process").mean_ms;
  s.optimizer_ms_avg = core::Profiler::instance().get("lba.run").mean_ms;
  s.map = impl_->map_service.summary();
  return s;
}
const Config& System::config() const { return impl_->pipeline->config; }
std::string System::effectiveConfigYaml() const { return impl_->pipeline->config.toYaml(); }

}  // namespace mvslam
