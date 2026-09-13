#include "mvslam/api/system.hpp"

#include "api/map_service_impl.hpp"
#include "core/log.hpp"
#include "core/profiler.hpp"
#include "mvslam/api/errors.hpp"
#include "mvslam/api/version.hpp"
#include "pipelines/pipeline_factory.hpp"

namespace mvslam {

struct System::Impl {
  explicit Impl(Config cfg) : pipeline(pipelines::PipelineFactory::build(cfg)), map_service(*pipeline->maps) {}
  std::unique_ptr<pipelines::Pipeline> pipeline;
  api::MapServiceImpl map_service;
  std::vector<ImuSample> imu_buffer;
  Timestamp last_imu = 0, last_frame = 0;
  std::vector<PoseStamped> trajectory;
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
  p.frontend->setOptimizerBusy(p.optimizer && p.optimizer->busy());
  TrackResult r = p.frontend->process(input, kf);
  if (kf) p.mapper->submit(pipelines::MapperJob{std::move(*kf), {}});
  if (r.pose) impl_->trajectory.push_back(PoseStamped{r.timestamp, r.pose->T_wb});
  r.stats.frontend_ms = core::Profiler::instance().get("frontend.process").mean_ms;
  return r;
}

MapService& System::mapService() { return impl_->map_service; }
std::vector<PoseStamped> System::trajectory() const { return impl_->trajectory; }
SystemStats System::stats() const {
  SystemStats s;
  s.frontend_ms_avg = core::Profiler::instance().get("frontend.process").mean_ms;
  s.map = impl_->map_service.summary();
  return s;
}
const Config& System::config() const { return impl_->pipeline->config; }
std::string System::effectiveConfigYaml() const { return impl_->pipeline->config.toYaml(); }

}  // namespace mvslam
