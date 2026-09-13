#include "core/log.hpp"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <memory>
#include <vector>

namespace mvslam::core {

void initLogging(const std::string& level, bool to_stderr, const std::string& file) {
  std::vector<spdlog::sink_ptr> sinks;
  if (to_stderr) sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
  if (!file.empty()) sinks.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(file, true));
  auto logger = std::make_shared<spdlog::logger>("mvslam", sinks.begin(), sinks.end());
  logger->set_level(spdlog::level::from_str(level));
  logger->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%t] %v");
  spdlog::set_default_logger(logger);
}

}  // namespace mvslam::core
