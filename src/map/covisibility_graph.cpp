#include "map/covisibility_graph.hpp"

#include <algorithm>

namespace mvslam::map {

const std::map<core::KeyframeId, int> CovisibilityGraph::kEmpty;

void CovisibilityGraph::setWeight(core::KeyframeId a, core::KeyframeId b, int w) {
  if (a == b) return;
  if (w <= 0) { adj_[a].erase(b); adj_[b].erase(a); return; }
  adj_[a][b] = w;
  adj_[b][a] = w;
}

void CovisibilityGraph::erase(core::KeyframeId kf) {
  auto it = adj_.find(kf);
  if (it == adj_.end()) return;
  for (const auto& [other, _] : it->second) adj_[other].erase(kf);
  adj_.erase(it);
}

int CovisibilityGraph::weight(core::KeyframeId a, core::KeyframeId b) const {
  auto it = adj_.find(a);
  if (it == adj_.end()) return 0;
  auto jt = it->second.find(b);
  return jt == it->second.end() ? 0 : jt->second;
}

std::vector<std::pair<core::KeyframeId, int>> CovisibilityGraph::covisibles(core::KeyframeId kf, int min_weight) const {
  std::vector<std::pair<core::KeyframeId, int>> out;
  auto it = adj_.find(kf);
  if (it == adj_.end()) return out;
  for (const auto& [other, w] : it->second) if (w >= min_weight) out.emplace_back(other, w);
  std::sort(out.begin(), out.end(), [](const auto& x, const auto& y) { return x.second > y.second; });
  return out;
}

const std::map<core::KeyframeId, int>& CovisibilityGraph::neighbours(core::KeyframeId kf) const {
  auto it = adj_.find(kf);
  return it == adj_.end() ? kEmpty : it->second;
}

}  // namespace mvslam::map
