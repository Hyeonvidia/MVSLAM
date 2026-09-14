#pragma once
// 3-D/3-D alignment for loop verification and map merging: RANSAC over 3-point Umeyama fits
// (SE3, or Sim3 for monocular maps), inliers by Euclidean distance, final fit on all inliers.
#include <vector>

#include "core/math/se3.hpp"

namespace mvslam::backend::loop {

struct AlignmentOptions { bool with_scale = false; int iterations = 200; double inlier_dist_m = 0.1; int min_inliers = 12; unsigned seed = 7; };
struct Alignment { bool ok = false; core::SE3 T_dst_src; double scale = 1.0; std::vector<int> inliers; };

// dst ~= scale * R * src + t (scale = 1 unless with_scale).
Alignment alignPointsRansac(const std::vector<core::Vec3>& src, const std::vector<core::Vec3>& dst, const AlignmentOptions& opt);

}  // namespace mvslam::backend::loop
