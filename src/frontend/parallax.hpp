#pragma once
// Median pixel parallax of the tracks shared by two frames, with the rotation between them
// compensated (OV2SLAM computeParallax with do_unrot): a pure rotation yields ~0 parallax.
#include <algorithm>
#include <unordered_map>
#include <vector>

#include "frontend/interfaces.hpp"
#include "sensors/camera_rig.hpp"

namespace mvslam::frontend {

inline double medianRotationCompensatedParallax(const Frame& ref, const Frame& cur, const sensors::CameraRig& rig) {
  std::unordered_map<core::TrackId, core::Vec2> ref_uv;
  for (const auto& t : ref.tracks) ref_uv.emplace(t.id, t.uv);
  const auto& cam = rig.camera(0);
  // Rotation of camera 0 from ref to cur: R_c_cur_c_ref = R_cb * R_bw(cur) * R_wb(ref) * R_bc
  const core::Mat3 R = (rig.T_cb(0) * cur.T_wb.inverse() * ref.T_wb * rig.T_bc(0)).rotationMatrix();
  std::vector<double> d;
  d.reserve(cur.tracks.size());
  for (const auto& t : cur.tracks) {
    auto it = ref_uv.find(t.id);
    if (it == ref_uv.end()) continue;
    const core::Vec3 ray = cam.unprojectRaw(cam.undistortToRaw(it->second));  // ref undistorted pixel -> ray
    const core::Vec3 rot = R * ray;
    if (rot.z() <= 1e-6) continue;
    const core::Vec2 uv_ref_in_cur = cam.project(rot);
    d.push_back((t.uv - uv_ref_in_cur).norm());
  }
  if (d.empty()) return 0.0;
  std::nth_element(d.begin(), d.begin() + static_cast<long>(d.size() / 2), d.end());
  return d[d.size() / 2];
}

}  // namespace mvslam::frontend
