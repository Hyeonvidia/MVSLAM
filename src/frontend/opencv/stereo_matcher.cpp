#include "frontend/opencv/stereo_matcher.hpp"

#include <opencv2/video/tracking.hpp>

#include "core/log.hpp"

namespace mvslam::frontend::cv_impl {

namespace {
// Linear two-ray triangulation in camera-0 coordinates: ray0 = [x0,y0,1] from camera 0, ray1 from
// camera 1 expressed in camera 0 through T_c0c1. Returns depth along ray0 (<0 if degenerate).
double triangulateDepth(const core::Vec3& ray0, const core::Vec3& ray1_c1, const core::SE3& T_c0c1, double& depth1) {
  const core::Vec3 d1 = T_c0c1.rotationMatrix() * ray1_c1;
  const core::Vec3 o1 = T_c0c1.translation();
  // min || l0*ray0 - (o1 + l1*d1) ||  -> 2x2 normal equations
  const double a = ray0.dot(ray0), b = ray0.dot(d1), c = d1.dot(d1), d = ray0.dot(o1), e = d1.dot(o1);
  const double det = a * c - b * b;
  if (std::abs(det) < 1e-12) { depth1 = -1; return -1; }
  const double l0 = (c * d - b * e) / det;
  depth1 = (b * d - a * e) / det;
  return l0;
}
}  // namespace

KltStereoMatcher::KltStereoMatcher(const FrontendConfig& c, const sensors::CameraRig& rig)
    : c_(c), rig_(rig), preprocessor_(c) {
  T_c1c0_ = rig.T_cb(1) * rig.T_bc(0);
  const auto& c0 = rig.camera(0);
  const auto& c1 = rig.camera(1);
  core::Mat3 K0, K1;
  K0 << c0.fx(), 0, c0.cx(), 0, c0.fy(), c0.cy(), 0, 0, 1;
  K1 << c1.fx(), 0, c1.cx(), 0, c1.fy(), c1.cy(), 0, 0, 1;
  const core::Mat3 E = core::SO3::hat(T_c1c0_.translation()) * T_c1c0_.rotationMatrix();
  F_10_ = K1.inverse().transpose() * E * K0.inverse();
}

std::vector<DepthMeasurement> KltStereoMatcher::measure(const KeyframeRequest& req) {
  const auto& tracks = req.frame.tracks;
  std::vector<DepthMeasurement> out(tracks.size());
  if (req.right_raw.empty() || !req.image_state) return out;
  const auto* left = dynamic_cast<const OpenCvImageState*>(req.image_state.get());
  if (!left) return out;
  auto right_state = preprocessor_.process(req.right_raw.view());
  const auto& right = static_cast<const OpenCvImageState&>(*right_state);
  const auto& c0 = rig_.camera(0);
  const auto& c1 = rig_.camera(1);
  const core::SE3 T_c1w = rig_.T_cb(1) * req.frame.T_wb.inverse();
  const core::SE3 T_c0c1 = T_c1c0_.inverse();

  // Initial guesses in the right image.
  std::vector<cv::Point2f> pl, pr;
  std::vector<int> levels;
  pl.reserve(tracks.size()); pr.reserve(tracks.size());
  for (const auto& t : tracks) {
    pl.emplace_back(static_cast<float>(t.uv_raw.x()), static_cast<float>(t.uv_raw.y()));
    cv::Point2f guess = pl.back();
    int lvl = c_.klt_levels;
    if (t.is3d) {
      const core::Vec3 pc1 = T_c1w * t.X_w;
      if (pc1.z() > 0) {
        const core::Vec2 uv1 = c1.undistortToRaw(c1.project(pc1));
        if (c1.inImage(uv1, 1)) { guess = cv::Point2f(static_cast<float>(uv1.x()), static_cast<float>(uv1.y())); lvl = 1; }
      }
    }
    pr.push_back(guess);
    levels.push_back(lvl);
  }
  auto klt = [&](int maxLevel, std::vector<int>& sel) {
    if (sel.empty()) return;
    std::vector<cv::Point2f> a, b;
    for (int i : sel) { a.push_back(pl[static_cast<std::size_t>(i)]); b.push_back(pr[static_cast<std::size_t>(i)]); }
    std::vector<unsigned char> status, bstatus;
    std::vector<float> err, berr;
    const cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    cv::calcOpticalFlowPyrLK(left->pyramid, right.pyramid, a, b, status, err, cv::Size(c_.klt_window, c_.klt_window), maxLevel, crit,
                             cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
    std::vector<cv::Point2f> back = a;  // backward pass starts from the original left position
    cv::calcOpticalFlowPyrLK(right.pyramid, left->pyramid, b, back, bstatus, berr, cv::Size(c_.klt_window, c_.klt_window), 0, crit,
                             cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
    for (std::size_t k = 0; k < sel.size(); ++k) {
      const int i = sel[k];
      ++stats_.attempted;
      const bool klt_ok = status[k] && bstatus[k] && static_cast<double>(err[k]) <= c_.klt_max_err &&
                          b[k].x >= 1 && b[k].y >= 1 && b[k].x < static_cast<float>(right.gray.cols - 1) && b[k].y < static_cast<float>(right.gray.rows - 1);
      if (!klt_ok) { ++stats_.rejected_klt; continue; }
      if (cv::norm(back[k] - a[k]) > c_.klt_fb_max_px) { ++stats_.rejected_fb; continue; }
      const core::Vec2 uv0 = tracks[static_cast<std::size_t>(i)].uv;
      const core::Vec2 uv1 = c1.undistort(core::Vec2(b[k].x, b[k].y));
      // Epipolar consistency (Sampson distance on undistorted pixels).
      const core::Vec3 x0(uv0.x(), uv0.y(), 1), x1(uv1.x(), uv1.y(), 1);
      const core::Vec3 Fx0 = F_10_ * x0, Ftx1 = F_10_.transpose() * x1;
      const double num = x1.dot(Fx0);
      const double den = Fx0.head<2>().squaredNorm() + Ftx1.head<2>().squaredNorm();
      if (den <= 0 || (num * num) / den > c_.stereo_epipolar_px * c_.stereo_epipolar_px) { ++stats_.rejected_epipolar; continue; }
      double d1 = -1;
      const double d0 = triangulateDepth(c0.unprojectUndistorted(uv0), c1.unprojectUndistorted(uv1), T_c0c1, d1);
      if (d0 < c_.min_depth_m || d0 > c_.max_depth_m || d1 <= 0) { ++stats_.rejected_depth; continue; }
      // Reprojection check in the right view.
      const core::Vec3 p0 = d0 * c0.unprojectUndistorted(uv0);
      const core::Vec3 p1 = T_c1c0_ * p0;
      if (p1.z() <= 0 || (c1.project(p1) - uv1).norm() > c_.triangulationReprojPx()) { ++stats_.rejected_reproj; continue; }
      ++stats_.accepted;
      out[static_cast<std::size_t>(i)] = DepthMeasurement{d0, true, uv1};
    }
  };
  std::vector<int> sel_projected, sel_full;
  for (std::size_t i = 0; i < tracks.size(); ++i) (levels[i] == 1 ? sel_projected : sel_full).push_back(static_cast<int>(i));
  klt(1, sel_projected);
  std::vector<int> retry;
  for (int i : sel_projected) if (out[static_cast<std::size_t>(i)].depth < 0) { pr[static_cast<std::size_t>(i)] = pl[static_cast<std::size_t>(i)]; retry.push_back(i); }
  sel_full.insert(sel_full.end(), retry.begin(), retry.end());
  klt(c_.klt_levels, sel_full);
  MVSLAM_DEBUG("stereo: attempted={} klt={} fb={} epi={} depth={} reproj={} accepted={}", stats_.attempted, stats_.rejected_klt,
               stats_.rejected_fb, stats_.rejected_epipolar, stats_.rejected_depth, stats_.rejected_reproj, stats_.accepted);
  stats_ = {};
  return out;
}

}  // namespace mvslam::frontend::cv_impl
