#pragma once
// Factory: SensorMode x InertialMode x ExecutionMode -> strategy objects and workers.
// This is the only place that switches on the mode (docs/design/01-architecture.md §2).
#include <memory>

#include "pipelines/pipeline.hpp"

namespace mvslam::pipelines {

class PipelineFactory {
 public:
  static std::unique_ptr<Pipeline> build(const Config& cfg);
};

}  // namespace mvslam::pipelines
