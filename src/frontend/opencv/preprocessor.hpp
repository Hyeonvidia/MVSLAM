#pragma once
// OpenCV implementation of ImagePreprocessor: gray conversion, optional CLAHE, KLT pyramid.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "frontend/interfaces.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::frontend::cv_impl {

struct OpenCvImageState final : ImageState {
  cv::Mat gray;                  // CLAHE-enhanced (detection input)
  cv::Mat raw;                   // untouched gray (descriptor input)
  std::vector<cv::Mat> pyramid;  // cv::buildOpticalFlowPyramid output
};

class OpenCvPreprocessor final : public ImagePreprocessor {
 public:
  explicit OpenCvPreprocessor(const FrontendConfig& c);
  std::unique_ptr<ImageState> process(const ImageView& image) override;
 private:
  FrontendConfig c_;
  cv::Ptr<cv::CLAHE> clahe_;
};

cv::Mat wrap(const ImageView& v);  // zero-copy view -> cv::Mat (caller keeps `v` alive)

}  // namespace mvslam::frontend::cv_impl
