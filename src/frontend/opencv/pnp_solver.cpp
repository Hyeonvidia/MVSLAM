#include "frontend/opencv/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <unordered_set>

namespace mvslam::frontend::cv_impl {

bool P3pRansacSolver::solve(const Frame& frame, core::SE3& T_wb, std::vector<int>& outlier_indices) {
  std::vector<cv::Point3d> obj;
  std::vector<cv::Point2d> img;
  std::vector<int> idx;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const auto& t = frame.tracks[i];
    if (!t.is3d) continue;
    obj.emplace_back(t.X_w.x(), t.X_w.y(), t.X_w.z());
    img.emplace_back(t.uv.x(), t.uv.y());
    idx.push_back(static_cast<int>(i));
  }
  if (static_cast<int>(idx.size()) < std::max(4, c_.p3p_min_tracks)) return false;
  const auto& cam = rig_.camera(0);
  cv::Mat K = (cv::Mat_<double>(3, 3) << cam.fx(), 0, cam.cx(), 0, cam.fy(), cam.cy(), 0, 0, 1);
  cv::Mat rvec, tvec, inliers;
  const bool ok = cv::solvePnPRansac(obj, img, K, cv::noArray(), rvec, tvec, false, c_.ransac_iters, static_cast<float>(c_.ransac_err_px),
                                     0.99, inliers, cv::SOLVEPNP_AP3P);
  if (!ok || inliers.rows < std::max(4, c_.p3p_min_tracks)) return false;
  cv::Mat R;
  cv::Rodrigues(rvec, R);
  core::Mat3 R_cw;
  core::Vec3 t_cw;
  cv::cv2eigen(R, R_cw);
  cv::cv2eigen(tvec, t_cw);
  const core::SE3 T_cw(Eigen::Quaterniond(R_cw).normalized(), t_cw);
  T_wb = (rig_.T_bc(0) * T_cw).inverse();
  std::unordered_set<int> in;
  for (int i = 0; i < inliers.rows; ++i) in.insert(inliers.at<int>(i));
  outlier_indices.clear();
  for (int i = 0; i < static_cast<int>(idx.size()); ++i) if (!in.count(i)) outlier_indices.push_back(idx[static_cast<std::size_t>(i)]);
  return true;
}

}  // namespace mvslam::frontend::cv_impl
