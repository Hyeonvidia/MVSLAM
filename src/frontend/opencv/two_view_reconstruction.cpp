#include "frontend/opencv/two_view_reconstruction.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <cmath>

#include "core/log.hpp"

namespace mvslam::frontend::cv_impl {

namespace {
constexpr double kThH = 5.991, kThF = 3.841, kThScore = 5.991;

// ORB-SLAM symmetric transfer scores: every inlier correspondence contributes (th - chi2).
double scoreHomography(const core::Mat3& H, const std::vector<TwoViewMatch>& m, double sigma2, std::vector<bool>& inlier) {
  const core::Mat3 Hinv = H.inverse();
  double score = 0;
  inlier.assign(m.size(), false);
  for (std::size_t i = 0; i < m.size(); ++i) {
    const core::Vec3 x1(m[i].p1.x(), m[i].p1.y(), 1), x2(m[i].p2.x(), m[i].p2.y(), 1);
    core::Vec3 x2in1 = Hinv * x2;
    x2in1 /= x2in1.z();
    core::Vec3 x1in2 = H * x1;
    x1in2 /= x1in2.z();
    const double c1 = (x2in1.head<2>() - x1.head<2>()).squaredNorm() / sigma2;
    const double c2 = (x1in2.head<2>() - x2.head<2>()).squaredNorm() / sigma2;
    bool ok = true;
    if (c1 <= kThH) score += kThH - c1; else ok = false;
    if (c2 <= kThH) score += kThH - c2; else ok = false;
    inlier[i] = ok;
  }
  return score;
}

double scoreFundamental(const core::Mat3& F, const std::vector<TwoViewMatch>& m, double sigma2, std::vector<bool>& inlier) {
  double score = 0;
  inlier.assign(m.size(), false);
  for (std::size_t i = 0; i < m.size(); ++i) {
    const core::Vec3 x1(m[i].p1.x(), m[i].p1.y(), 1), x2(m[i].p2.x(), m[i].p2.y(), 1);
    const core::Vec3 l2 = F * x1, l1 = F.transpose() * x2;
    const double d2 = x2.dot(l2), d1 = x1.dot(l1);
    const double c1 = d2 * d2 / (l2.x() * l2.x() + l2.y() * l2.y()) / sigma2;
    const double c2 = d1 * d1 / (l1.x() * l1.x() + l1.y() * l1.y()) / sigma2;
    bool ok = true;
    if (c1 <= kThF) score += kThScore - c1; else ok = false;
    if (c2 <= kThF) score += kThScore - c2; else ok = false;
    inlier[i] = ok;
  }
  return score;
}

// Triangulates the inliers for one (R, t) hypothesis (camera 2 from camera 1); returns #good points.
int checkHypothesis(const core::Mat3& R, const core::Vec3& t, const std::vector<TwoViewMatch>& m, const std::vector<bool>& inlier,
                    const sensors::ICameraModel& cam, double sigma2, std::vector<core::Vec3>& X1, std::vector<bool>& good,
                    double& parallax_deg) {
  X1.assign(m.size(), core::Vec3::Zero());
  good.assign(m.size(), false);
  std::vector<double> cos_par;
  const core::Vec3 O2 = -R.transpose() * t;  // camera-2 centre in camera-1 coordinates
  int n = 0;
  for (std::size_t i = 0; i < m.size(); ++i) {
    if (!inlier[i]) continue;
    const core::Vec3 r1 = cam.unprojectUndistorted(m[i].p1), r2 = cam.unprojectUndistorted(m[i].p2);
    const core::Vec3 d = R.transpose() * r2;  // ray 2 expressed in camera 1: O2 + d2 * d
    const double a = r1.dot(r1), b = r1.dot(d), c = d.dot(d), e = r1.dot(O2), f = d.dot(O2);
    const double det = a * c - b * b;
    if (std::abs(det) < 1e-12) continue;
    const double d1 = (c * e - b * f) / det, d2 = (b * e - a * f) / det;
    if (d1 <= 0 || d2 <= 0) continue;
    const core::Vec3 X = 0.5 * (d1 * r1 + (O2 + d2 * d));
    const core::Vec3 X2 = R * X + t;
    if (X.z() <= 0 || X2.z() <= 0) continue;
    if ((cam.project(X) - m[i].p1).squaredNorm() > 4 * sigma2) continue;
    if ((cam.project(X2) - m[i].p2).squaredNorm() > 4 * sigma2) continue;
    cos_par.push_back(X.normalized().dot((X - O2).normalized()));
    X1[i] = X;
    good[i] = true;
    ++n;
  }
  parallax_deg = 0;
  if (!cos_par.empty()) {
    std::sort(cos_par.begin(), cos_par.end());  // ascending cosine == descending parallax
    const std::size_t idx = std::min(cos_par.size() - 1, static_cast<std::size_t>(50));
    parallax_deg = std::acos(std::clamp(cos_par[idx], -1.0, 1.0)) * 180.0 / M_PI;
  }
  return n;
}
}  // namespace

std::optional<TwoViewResult> reconstructTwoView(const std::vector<TwoViewMatch>& m, const sensors::ICameraModel& cam,
                                                const TwoViewOptions& opt) {
  if (m.size() < 8) return std::nullopt;
  std::vector<cv::Point2f> p1, p2;
  p1.reserve(m.size());
  p2.reserve(m.size());
  for (const auto& x : m) {
    p1.emplace_back(static_cast<float>(x.p1.x()), static_cast<float>(x.p1.y()));
    p2.emplace_back(static_cast<float>(x.p2.x()), static_cast<float>(x.p2.y()));
  }
  const double sigma2 = opt.sigma_px * opt.sigma_px;
  const cv::Mat K = (cv::Mat_<double>(3, 3) << cam.fx(), 0, cam.cx(), 0, cam.fy(), cam.cy(), 0, 0, 1);
  // H by RANSAC; E by Nister's 5-point RANSAC (more accurate than an 8-point F), scored as F = K^-T E K^-1.
  const cv::Mat Hcv = cv::findHomography(p1, p2, cv::RANSAC, 3.0 * opt.sigma_px, cv::noArray(), opt.ransac_iters, 0.999);
  // USAC_ACCURATE: LO-RANSAC with a final least-squares polish on all inliers (a plain RANSAC minimal-sample
  // solution carries ~0.4 deg rotation error at 0.2 px noise, which triangulation amplifies).
  const cv::Mat Ecv = cv::findEssentialMat(p1, p2, K, cv::USAC_ACCURATE, 0.999, 1.0 * opt.sigma_px, opt.ransac_iters, cv::noArray());
  if (Hcv.empty() || Ecv.empty() || Ecv.rows != 3) return std::nullopt;
  core::Mat3 H, E, Km;
  cv::cv2eigen(Hcv, H);
  cv::cv2eigen(Ecv, E);
  cv::cv2eigen(K, Km);
  const core::Mat3 Kinv = Km.inverse();
  const core::Mat3 F = Kinv.transpose() * E * Kinv;
  std::vector<bool> inl_h, inl_f;
  const double s_h = scoreHomography(H, m, sigma2, inl_h), s_f = scoreFundamental(F, m, sigma2, inl_f);
  const double r_h = s_h / std::max(1e-9, s_h + s_f);
  const bool use_h = r_h > opt.h_selection_threshold;

  std::vector<std::pair<core::Mat3, core::Vec3>> hypotheses;
  if (use_h) {  // planar / low parallax: homography decomposition (up to 4 solutions)
    std::vector<cv::Mat> Rs, ts, ns;
    cv::decomposeHomographyMat(Hcv, K, Rs, ts, ns);
    for (std::size_t k = 0; k < Rs.size(); ++k) {
      core::Mat3 R;
      core::Vec3 t;
      cv::cv2eigen(Rs[k], R);
      cv::cv2eigen(ts[k], t);
      if (t.norm() < 1e-9) continue;
      hypotheses.emplace_back(R, t.normalized());
    }
  } else {
    cv::Mat R1, R2, t;
    cv::decomposeEssentialMat(Ecv, R1, R2, t);
    core::Mat3 Ra, Rb;
    core::Vec3 tt;
    cv::cv2eigen(R1, Ra);
    cv::cv2eigen(R2, Rb);
    cv::cv2eigen(t, tt);
    hypotheses = {{Ra, tt}, {Ra, -tt}, {Rb, tt}, {Rb, -tt}};
  }
  const std::vector<bool>& inlier = use_h ? inl_h : inl_f;
  int n_inl = 0;
  for (bool b : inlier) n_inl += b ? 1 : 0;

  TwoViewResult best;
  int best_n = 0, second_n = 0;
  double best_parallax = 0;
  for (const auto& [R, t] : hypotheses) {
    std::vector<core::Vec3> X;
    std::vector<bool> good;
    double par = 0;
    const int n = checkHypothesis(R, t, m, inlier, cam, sigma2, X, good, par);
    if (n > best_n) {
      second_n = best_n;
      best_n = n;
      best.T_c2c1 = core::SE3(Eigen::Quaterniond(R).normalized(), t);
      best.X_c1 = std::move(X);
      best.triangulated = std::move(good);
      best_parallax = par;
    } else if (n > second_n) {
      second_n = n;
    }
  }
  const int min_good = std::max(opt.min_triangulated, static_cast<int>(opt.min_good_ratio * n_inl));
  const bool ambiguous = second_n > 0.7 * best_n;
  if (best_n < min_good || ambiguous || best_parallax < opt.min_parallax_deg) {
    MVSLAM_DEBUG("two-view: rejected (R_H={:.2f} model={} inliers={} best={} second={} parallax={:.2f}deg)", r_h, use_h ? "H" : "F",
                 n_inl, best_n, second_n, best_parallax);
    return std::nullopt;
  }
  best.r_h = r_h;
  best.inliers = n_inl;
  best.good = best_n;
  best.used_homography = use_h;
  return best;
}

}  // namespace mvslam::frontend::cv_impl
