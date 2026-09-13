#pragma once
// Two-view reconstruction for the monocular bootstrap (ORB-SLAM TwoViewReconstruction semantics):
// homography (RANSAC) and essential matrix (5-point RANSAC) estimated in parallel, model chosen by
// R_H = S_H / (S_H + S_F) with ORB-SLAM's symmetric transfer scores,
// relative pose recovered from the chosen model, linear triangulation with cheirality / reprojection /
// parallax checks. Pure geometry on undistorted pixels; no map or tracker dependencies (unit-testable).
#include <optional>
#include <vector>

#include "core/ids.hpp"
#include "core/math/se3.hpp"
#include "sensors/camera_model.hpp"

namespace mvslam::frontend::cv_impl {

struct TwoViewMatch { core::TrackId id; core::Vec2 p1, p2; };  // undistorted pixels in view 1 / view 2

struct TwoViewOptions {
  double sigma_px = 1.0;              // measurement noise for the chi2 gates
  int ransac_iters = 200;
  double min_parallax_deg = 1.0;      // 50th-best triangulation parallax must exceed this
  int min_triangulated = 50;
  double min_good_ratio = 0.9;        // good points / model inliers
  double h_selection_threshold = 0.45; // R_H above this -> homography (planar / low parallax)
};

struct TwoViewResult {
  core::SE3 T_c2c1;                   // view 1 -> view 2, unit translation norm
  std::vector<bool> triangulated;     // per match
  std::vector<core::Vec3> X_c1;       // per match (valid where triangulated)
  double r_h = 0;
  int inliers = 0, good = 0;
  bool used_homography = false;
};

std::optional<TwoViewResult> reconstructTwoView(const std::vector<TwoViewMatch>& matches, const sensors::ICameraModel& cam,
                                                const TwoViewOptions& opt);

}  // namespace mvslam::frontend::cv_impl
