#include "frontend/opencv/klt_tracker.hpp"

#include <opencv2/video/tracking.hpp>

#include "frontend/opencv/preprocessor.hpp"

namespace mvslam::frontend::cv_impl {

namespace {
// Forward-backward pyramidal LK. Returns per-point status after both checks.
void fbKlt(const std::vector<cv::Mat>& prev_pyr, const std::vector<cv::Mat>& cur_pyr, int win, int levels,
           double max_err, double fb_max, const std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& cur_pts,
           std::vector<unsigned char>& status) {
  if (prev_pts.empty()) return;
  std::vector<float> err;
  const cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
  cv::calcOpticalFlowPyrLK(prev_pyr, cur_pyr, prev_pts, cur_pts, status, err, cv::Size(win, win), levels, crit,
                           cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
  std::vector<cv::Point2f> back = prev_pts;  // backward pass refines from the original position (level 0 only)
  std::vector<unsigned char> bstatus;
  std::vector<float> berr;
  cv::calcOpticalFlowPyrLK(cur_pyr, prev_pyr, cur_pts, back, bstatus, berr, cv::Size(win, win), 0, crit,
                           cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_LK_GET_MIN_EIGENVALS, 1e-4);
  const int w = cur_pyr[0].cols, h = cur_pyr[0].rows;
  for (std::size_t i = 0; i < prev_pts.size(); ++i) {
    bool ok = status[i] && bstatus[i] && static_cast<double>(err[i]) <= max_err;
    if (ok) ok = cv::norm(back[i] - prev_pts[i]) <= fb_max;
    if (ok) ok = cur_pts[i].x >= 1 && cur_pts[i].y >= 1 && cur_pts[i].x < w - 1 && cur_pts[i].y < h - 1;
    status[i] = ok ? 1 : 0;
  }
}
}  // namespace

IFeatureTracker::Stats KltTracker::track(const ImageState& prev_s, const ImageState& cur_s, const Frame& prev,
                                         Frame& cur, const core::SE3& predicted_T_wb) {
  Stats st;
  const auto& p = static_cast<const OpenCvImageState&>(prev_s);
  const auto& c = static_cast<const OpenCvImageState&>(cur_s);
  const auto& cam = rig_.camera(0);
  const core::SE3 T_cw = rig_.T_cb(0) * predicted_T_wb.inverse();

  // Stage 1: 3D tracks with projected initial guess, 2 pyramid levels.
  std::vector<std::size_t> idx3d, idx2d;
  std::vector<cv::Point2f> prev3d, cur3d;
  for (std::size_t i = 0; i < prev.tracks.size(); ++i) {
    const auto& t = prev.tracks[i];
    if (t.is3d) {
      const core::Vec3 pc = T_cw * t.X_w;
      if (pc.z() > 0) {
        const core::Vec2 uv = cam.project(pc);
        if (cam.inImage(uv, 1)) {
          idx3d.push_back(i);
          prev3d.emplace_back(static_cast<float>(t.uv_raw.x()), static_cast<float>(t.uv_raw.y()));
          cur3d.emplace_back(static_cast<float>(uv.x()), static_cast<float>(uv.y()));
          continue;
        }
      }
    }
    idx2d.push_back(i);
  }
  std::vector<unsigned char> s3d;
  fbKlt(p.pyramid, c.pyramid, c_.klt_window, 1, c_.klt_max_err, c_.klt_fb_max_px, prev3d, cur3d, s3d);
  st.attempted3d = static_cast<int>(idx3d.size());
  cur.tracks.clear();
  for (std::size_t k = 0; k < idx3d.size(); ++k) {
    if (s3d[k]) {
      Track t = prev.tracks[idx3d[k]];
      t.uv_raw = core::Vec2(cur3d[k].x, cur3d[k].y);
      t.uv = cam.undistort(t.uv_raw);
      t.age++;
      cur.tracks.push_back(t);
      st.survived3d++;
    } else {
      idx2d.push_back(idx3d[k]);  // retry on the full pyramid
    }
  }
  // Stage 2: 2D tracks (and 3D failures), full pyramid, initial guess = previous position.
  std::vector<cv::Point2f> prev2d, cur2d;
  for (auto i : idx2d) {
    const auto& t = prev.tracks[i];
    prev2d.emplace_back(static_cast<float>(t.uv_raw.x()), static_cast<float>(t.uv_raw.y()));
  }
  cur2d = prev2d;
  std::vector<unsigned char> s2d;
  fbKlt(p.pyramid, c.pyramid, c_.klt_window, c_.klt_levels, c_.klt_max_err, c_.klt_fb_max_px, prev2d, cur2d, s2d);
  st.attempted2d = static_cast<int>(idx2d.size());
  for (std::size_t k = 0; k < idx2d.size(); ++k) {
    if (!s2d[k]) continue;
    Track t = prev.tracks[idx2d[k]];
    t.uv_raw = core::Vec2(cur2d[k].x, cur2d[k].y);
    t.uv = cam.undistort(t.uv_raw);
    t.age++;
    cur.tracks.push_back(t);
    st.survived2d++;
  }
  return st;
}

}  // namespace mvslam::frontend::cv_impl
