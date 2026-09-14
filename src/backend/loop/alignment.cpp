#include "backend/loop/alignment.hpp"

#include <Eigen/Geometry>
#include <random>

namespace mvslam::backend::loop {

namespace {
bool fit(const std::vector<core::Vec3>& src, const std::vector<core::Vec3>& dst, const std::vector<int>& idx, bool with_scale, core::SE3& T,
         double& s) {
  Eigen::Matrix<double, 3, Eigen::Dynamic> A(3, idx.size()), B(3, idx.size());
  for (std::size_t k = 0; k < idx.size(); ++k) { A.col(static_cast<Eigen::Index>(k)) = src[static_cast<std::size_t>(idx[k])]; B.col(static_cast<Eigen::Index>(k)) = dst[static_cast<std::size_t>(idx[k])]; }
  const Eigen::Matrix4d M = Eigen::umeyama(A, B, with_scale);
  const core::Mat3 sR = M.block<3, 3>(0, 0);
  s = with_scale ? std::cbrt(sR.determinant()) : 1.0;
  if (!std::isfinite(s) || s <= 0) return false;
  const core::Mat3 R = sR / s;
  T = core::SE3(core::SO3(Eigen::Quaterniond(R).normalized()), M.block<3, 1>(0, 3));
  return true;
}
}  // namespace

Alignment alignPointsRansac(const std::vector<core::Vec3>& src, const std::vector<core::Vec3>& dst, const AlignmentOptions& opt) {
  Alignment best;
  const int n = static_cast<int>(std::min(src.size(), dst.size()));
  if (n < 3) return best;
  std::mt19937 rng(opt.seed);
  std::uniform_int_distribution<int> pick(0, n - 1);
  const double d2 = opt.inlier_dist_m * opt.inlier_dist_m;
  for (int it = 0; it < opt.iterations; ++it) {
    std::vector<int> sample;
    while (static_cast<int>(sample.size()) < 3) {
      const int i = pick(rng);
      if (std::find(sample.begin(), sample.end(), i) == sample.end()) sample.push_back(i);
    }
    core::SE3 T;
    double s = 1.0;
    if (!fit(src, dst, sample, opt.with_scale, T, s)) continue;
    std::vector<int> inl;
    for (int i = 0; i < n; ++i) {
      const core::Vec3 p = s * (T.so3() * src[static_cast<std::size_t>(i)]) + T.translation();
      if ((p - dst[static_cast<std::size_t>(i)]).squaredNorm() <= d2) inl.push_back(i);
    }
    if (inl.size() > best.inliers.size()) { best.inliers = inl; best.T_dst_src = T; best.scale = s; }
  }
  if (static_cast<int>(best.inliers.size()) < opt.min_inliers) return best;
  core::SE3 T;
  double s = 1.0;
  if (fit(src, dst, best.inliers, opt.with_scale, T, s)) {  // refit on all inliers, then re-evaluate them
    std::vector<int> inl;
    for (int i = 0; i < n; ++i) {
      const core::Vec3 p = s * (T.so3() * src[static_cast<std::size_t>(i)]) + T.translation();
      if ((p - dst[static_cast<std::size_t>(i)]).squaredNorm() <= d2) inl.push_back(i);
    }
    if (inl.size() >= best.inliers.size()) { best.inliers = inl; best.T_dst_src = T; best.scale = s; }
  }
  best.ok = static_cast<int>(best.inliers.size()) >= opt.min_inliers;
  return best;
}

}  // namespace mvslam::backend::loop
