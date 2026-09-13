#pragma once
// Thin logging facade over spdlog. Only this header includes spdlog in src/ (swap-able sink).
#include <spdlog/spdlog.h>

#include <string>

namespace mvslam::core {
void initLogging(const std::string& level, bool to_stderr, const std::string& file);
}  // namespace mvslam::core

#define MVSLAM_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define MVSLAM_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define MVSLAM_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define MVSLAM_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define MVSLAM_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
