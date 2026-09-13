#include "frontend/opencv/essential_filter.hpp"

#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <unordered_map>

namespace mvslam::frontend::cv_impl {

int EssentialRansacFilter::filter(const Frame& ref, Frame& cur) {
  std::unordered_map<core::TrackId, core::Vec2> ref_uv;
  for (const auto& t : ref.tracks) ref_uv.emplace(t.id, t.uv);
  std::vector<cv::Point2f> p_ref, p_cur;
  std::vector<std::size_t> idx;
  std::vector<double> disp;
  for (std::size_t i = 0; i < cur.tracks.size(); ++i) {
    auto it = ref_uv.find(cur.tracks[i].id);
    if (it == ref_uv.end()) continue;
    p_ref.emplace_back(static_cast<float>(it->second.x()), static_cast<float>(it->second.y()));
    p_cur.emplace_back(static_cast<float>(cur.tracks[i].uv.x()), static_cast<float>(cur.tracks[i].uv.y()));
    idx.push_back(i);
    disp.push_back((cur.tracks[i].uv - it->second).norm());
  }
  if (idx.size() < 8) return 0;
  std::nth_element(disp.begin(), disp.begin() + static_cast<long>(disp.size() / 2), disp.end());
  if (disp[disp.size() / 2] < c_.outlier_parallax_gate_px) return 0;  // E is degenerate without translation

  const auto& cam = rig_.camera(0);
  cv::Mat K = (cv::Mat_<double>(3, 3) << cam.fx(), 0, cam.cx(), 0, cam.fy(), cam.cy(), 0, 0, 1);
  cv::Mat mask;
  cv::Mat E = cv::findEssentialMat(p_ref, p_cur, K, cv::RANSAC, 0.999, c_.ransac_err_px, c_.ransac_iters, mask);
  if (E.empty() || mask.empty()) return 0;
  int outliers = 0;
  for (int i = 0; i < mask.rows; ++i) if (mask.at<unsigned char>(i) == 0) ++outliers;
  if (outliers > static_cast<int>(idx.size()) / 2) return 0;  // degenerate: keep everything
  std::vector<bool> drop(cur.tracks.size(), false);
  for (int i = 0; i < mask.rows; ++i) if (mask.at<unsigned char>(i) == 0) drop[idx[static_cast<std::size_t>(i)]] = true;
  std::vector<Track> kept;
  kept.reserve(cur.tracks.size());
  for (std::size_t i = 0; i < cur.tracks.size(); ++i) if (!drop[i]) kept.push_back(cur.tracks[i]);
  cur.tracks.swap(kept);
  return outliers;
}

}  // namespace mvslam::frontend::cv_impl
