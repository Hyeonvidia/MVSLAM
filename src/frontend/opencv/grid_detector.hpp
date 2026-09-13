#pragma once
// Grid Shi-Tomasi detector for keyframes: one best corner per empty cell (+subpixel refinement),
// adaptive quality threshold. Cells already occupied by live tracks are skipped.
#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::frontend::cv_impl {

class GridShiTomasiDetector final : public IFeatureDetector {
 public:
  explicit GridShiTomasiDetector(const FrontendConfig& c) : c_(c) {}
  std::vector<core::Vec2> detect(const ImageState& image, const std::vector<core::Vec2>& existing, int max_new) override;
 private:
  FrontendConfig c_;
  double quality_ = 0.001;
};

}  // namespace mvslam::frontend::cv_impl
