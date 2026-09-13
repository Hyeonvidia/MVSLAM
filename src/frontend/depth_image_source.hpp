#pragma once
// RGB-D depth source: reads the registered depth image at each keypoint (no OpenCV needed).
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::frontend {

class DepthImageSource final : public IDepthSource {
 public:
  explicit DepthImageSource(const FrontendConfig& c) : c_(c) {}
  std::vector<DepthMeasurement> measure(const KeyframeRequest& req) override {
    std::vector<DepthMeasurement> out(req.frame.tracks.size());
    const auto& d = req.depth;
    if (d.empty()) return out;
    for (std::size_t i = 0; i < req.frame.tracks.size(); ++i) {
      const auto& uv = req.frame.tracks[i].uv_raw;
      const int x = static_cast<int>(uv.x()), y = static_cast<int>(uv.y());
      if (x < 0 || y < 0 || x >= d.width || y >= d.height) continue;
      double z = -1;
      const std::uint8_t* row = d.data.data() + static_cast<std::size_t>(y) * d.stride;
      if (d.format == PixelFormat::Depth16) {
        std::uint16_t v; std::memcpy(&v, row + static_cast<std::size_t>(x) * 2, 2);
        z = v * c_.rgbd_depth_scale;
      } else if (d.format == PixelFormat::DepthF32) {
        float v; std::memcpy(&v, row + static_cast<std::size_t>(x) * 4, 4);
        z = v;
      }
      if (z >= c_.min_depth_m && z <= c_.max_depth_m) out[i].depth = z;
    }
    return out;
  }
 private:
  FrontendConfig c_;
};

}  // namespace mvslam::frontend
