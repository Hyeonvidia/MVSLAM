#pragma once
// Covisibility weights between keyframes of ONE map, plus the essential subgraph query.
// Not thread-safe by itself: guarded by the owning Map's lock.
#include <map>
#include <unordered_map>
#include <vector>

#include "core/ids.hpp"

namespace mvslam::map {

class CovisibilityGraph {
 public:
  void setWeight(core::KeyframeId a, core::KeyframeId b, int w);
  void erase(core::KeyframeId kf);
  int weight(core::KeyframeId a, core::KeyframeId b) const;
  // Neighbours with weight >= min_weight, sorted by decreasing weight.
  std::vector<std::pair<core::KeyframeId, int>> covisibles(core::KeyframeId kf, int min_weight = 1) const;
  const std::map<core::KeyframeId, int>& neighbours(core::KeyframeId kf) const;

 private:
  std::unordered_map<core::KeyframeId, std::map<core::KeyframeId, int>> adj_;
  static const std::map<core::KeyframeId, int> kEmpty;
};

}  // namespace mvslam::map
