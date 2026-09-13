#include "backend/local_ba/window_selector.hpp"

#include <set>

namespace mvslam::backend {

BaWindow CovisibilityWindow::select(const map::Map& map, core::KeyframeId latest) const {
  BaWindow w;
  w.optimized.push_back(latest);
  int n = 1;
  for (const auto& [kf, weight] : map.covisibility().covisibles(latest, c_.lba_min_covisibility)) {
    if (n >= c_.lba_max_keyframes) break;
    w.optimized.push_back(kf);
    ++n;
  }
  // Fixed set: keyframes observing the window's map points but not in the window.
  std::set<core::KeyframeId> in(w.optimized.begin(), w.optimized.end()), fixed;
  for (auto kfid : w.optimized) {
    auto kf = map.keyframe(kfid);
    if (!kf) continue;
    for (auto mpid : kf->mapPointSlots()) {
      if (!mpid.valid()) continue;
      auto mp = map.mapPoint(mpid);
      if (!mp) continue;
      for (const auto& [obs_kf, _] : mp->observations()) if (!in.count(obs_kf)) fixed.insert(obs_kf);
    }
  }
  w.fixed.assign(fixed.begin(), fixed.end());
  return w;
}

BaWindow TemporalWindow::select(const map::Map& map, core::KeyframeId latest) const {
  BaWindow w;
  const int n_max = large_ ? c_.inertial_window_large : c_.inertial_window;
  auto kf = map.keyframe(latest);
  while (kf && static_cast<int>(w.optimized.size()) < n_max) {
    w.optimized.push_back(kf->id());
    kf = map.keyframe(kf->prevKf());
  }
  if (kf) w.fixed.push_back(kf->id());  // the keyframe just before the window anchors the chain
  return w;
}

}  // namespace mvslam::backend
