#pragma once
// Public: library, map-format and config-schema versions. Bump rules: docs/design/05-api-config.md §6.
#define MVSLAM_VERSION_MAJOR 0
#define MVSLAM_VERSION_MINOR 1
#define MVSLAM_VERSION_PATCH 0
namespace mvslam {
inline constexpr const char* kVersion = "0.1.0";
inline constexpr int kMapFormatVersion = 1;
inline constexpr int kConfigSchemaVersion = 1;
}  // namespace mvslam
