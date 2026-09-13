#include "frontend/opencv/grid_detector.hpp"

#include <opencv2/imgproc.hpp>

#include "frontend/opencv/preprocessor.hpp"

namespace mvslam::frontend::cv_impl {

std::vector<core::Vec2> GridShiTomasiDetector::detect(const ImageState& image, const std::vector<core::Vec2>& existing, int max_new) {
  const auto& s = static_cast<const OpenCvImageState&>(image);
  const int cell = c_.detector_cell_px;
  const int ncx = s.gray.cols / cell, ncy = s.gray.rows / cell;
  std::vector<unsigned char> occupied(static_cast<std::size_t>(ncx * ncy), 0);
  for (const auto& p : existing) {
    const int cx = static_cast<int>(p.x()) / cell, cy = static_cast<int>(p.y()) / cell;
    if (cx >= 0 && cy >= 0 && cx < ncx && cy < ncy) occupied[static_cast<std::size_t>(cy * ncx + cx)] = 1;
  }
  cv::Mat response;
  cv::cornerMinEigenVal(s.gray, response, 3, 3);
  double maxv = 0;
  cv::minMaxLoc(response, nullptr, &maxv);
  const double th = quality_ * maxv;
  std::vector<cv::Point2f> pts;
  int empty_cells = 0;
  for (int cy = 0; cy < ncy && static_cast<int>(pts.size()) < max_new; ++cy) {
    for (int cx = 0; cx < ncx && static_cast<int>(pts.size()) < max_new; ++cx) {
      if (occupied[static_cast<std::size_t>(cy * ncx + cx)]) continue;
      ++empty_cells;
      cv::Rect roi(cx * cell, cy * cell, cell, cell);
      double best = 0;
      cv::Point loc;
      cv::minMaxLoc(response(roi), nullptr, &best, nullptr, &loc);
      if (best > th) pts.emplace_back(static_cast<float>(roi.x + loc.x), static_cast<float>(roi.y + loc.y));
    }
  }
  if (!pts.empty() && c_.detector_subpix) {
    cv::cornerSubPix(s.gray, pts, cv::Size(3, 3), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));
  }
  // Adaptive threshold (OV2SLAM): too few detections -> relax, almost all cells fire -> tighten.
  if (empty_cells > 0) {
    const double ratio = static_cast<double>(pts.size()) / empty_cells;
    if (ratio < 0.33) quality_ *= 0.5; else if (ratio > 0.9) quality_ *= 1.5;
  }
  std::vector<core::Vec2> out;
  out.reserve(pts.size());
  for (const auto& p : pts) out.emplace_back(p.x, p.y);
  return out;
}

}  // namespace mvslam::frontend::cv_impl
