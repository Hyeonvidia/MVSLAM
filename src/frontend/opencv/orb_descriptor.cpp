#include "frontend/opencv/orb_descriptor.hpp"

#include <opencv2/imgproc.hpp>

#include "frontend/opencv/preprocessor.hpp"

namespace mvslam::frontend::cv_impl {

std::vector<std::uint8_t> OrbDescriptorExtractor::describe(const ImageView& raw, const std::vector<core::Vec2>& uv_raw) {
  std::vector<std::uint8_t> out(uv_raw.size() * 32, 0);
  if (uv_raw.empty()) return out;
  cv::Mat img = wrap(raw), gray;
  if (raw.format == PixelFormat::Rgb8) cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
  else if (raw.format == PixelFormat::Bgr8) cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  else gray = img;
  std::vector<cv::KeyPoint> kps;
  kps.reserve(uv_raw.size());
  for (std::size_t i = 0; i < uv_raw.size(); ++i) {
    cv::KeyPoint k(static_cast<float>(uv_raw[i].x()), static_cast<float>(uv_raw[i].y()), 31.f);
    k.class_id = static_cast<int>(i);
    kps.push_back(k);
  }
  cv::Mat desc;
  orb_->compute(gray, kps, desc);
  for (std::size_t r = 0; r < kps.size(); ++r) {
    const int idx = kps[r].class_id;
    if (idx < 0 || static_cast<std::size_t>(idx) >= uv_raw.size()) continue;
    std::memcpy(out.data() + static_cast<std::size_t>(idx) * 32, desc.ptr<std::uint8_t>(static_cast<int>(r)), 32);
  }
  return out;
}

}  // namespace mvslam::frontend::cv_impl
