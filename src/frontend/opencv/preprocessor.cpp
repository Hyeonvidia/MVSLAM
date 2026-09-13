#include "frontend/opencv/preprocessor.hpp"

#include <opencv2/video/tracking.hpp>

namespace mvslam::frontend::cv_impl {

cv::Mat wrap(const ImageView& v) {
  int type = CV_8UC1;
  switch (v.format) {
    case PixelFormat::Gray8: type = CV_8UC1; break;
    case PixelFormat::Rgb8: case PixelFormat::Bgr8: type = CV_8UC3; break;
    case PixelFormat::Depth16: type = CV_16UC1; break;
    case PixelFormat::DepthF32: type = CV_32FC1; break;
  }
  return cv::Mat(v.height, v.width, type, const_cast<std::uint8_t*>(v.data), static_cast<std::size_t>(v.stride));
}

OpenCvPreprocessor::OpenCvPreprocessor(const FrontendConfig& c) : c_(c) {
  if (c_.clahe) clahe_ = cv::createCLAHE(c_.clahe_clip, cv::Size(8, 8));
}

std::unique_ptr<ImageState> OpenCvPreprocessor::process(const ImageView& image) {
  auto st = std::make_unique<OpenCvImageState>();
  cv::Mat m = wrap(image);
  if (image.format == PixelFormat::Rgb8) cv::cvtColor(m, st->raw, cv::COLOR_RGB2GRAY);
  else if (image.format == PixelFormat::Bgr8) cv::cvtColor(m, st->raw, cv::COLOR_BGR2GRAY);
  else st->raw = m.clone();
  if (clahe_) clahe_->apply(st->raw, st->gray); else st->gray = st->raw;
  cv::buildOpticalFlowPyramid(st->gray, st->pyramid, cv::Size(c_.klt_window, c_.klt_window), c_.klt_levels);
  return st;
}

}  // namespace mvslam::frontend::cv_impl
