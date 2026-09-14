#include "map/hbst_place_recognizer.hpp"

#include <algorithm>
#include <bit>
#include <cstring>
#include <unordered_map>

namespace mvslam::map {

namespace {
inline bool bitOf(const std::array<std::uint8_t, 32>& d, int bit) { return (d[static_cast<std::size_t>(bit >> 3)] >> (bit & 7)) & 1; }
inline int hamming(const std::array<std::uint8_t, 32>& a, const std::array<std::uint8_t, 32>& b) {
  int d = 0;
  for (int i = 0; i < 32; i += 8) {
    std::uint64_t x, y;
    std::memcpy(&x, a.data() + i, 8);
    std::memcpy(&y, b.data() + i, 8);
    d += std::popcount(x ^ y);
  }
  return d;
}
inline bool allZero(const std::array<std::uint8_t, 32>& d) { for (auto b : d) if (b) return false; return true; }
}  // namespace

void HbstPlaceRecognizer::split(Node& n, int depth) {
  if (depth >= opt_.max_depth) return;
  // Most balanced bit over the leaf's descriptors.
  std::array<int, 256> ones{};
  for (auto i : n.items) for (int b = 0; b < 256; ++b) ones[static_cast<std::size_t>(b)] += bitOf(descs_[i], b) ? 1 : 0;
  const int half = static_cast<int>(n.items.size()) / 2;
  int best = -1, best_dev = 1 << 30;
  for (int b = 0; b < 256; ++b) {
    const int dev = std::abs(ones[static_cast<std::size_t>(b)] - half);
    if (dev < best_dev) { best_dev = dev; best = b; }
  }
  if (best < 0 || ones[static_cast<std::size_t>(best)] == 0 || ones[static_cast<std::size_t>(best)] == static_cast<int>(n.items.size())) return;  // degenerate leaf
  n.bit = best;
  n.child[0] = std::make_unique<Node>();
  n.child[1] = std::make_unique<Node>();
  for (auto i : n.items) n.child[bitOf(descs_[i], best) ? 1 : 0]->items.push_back(i);
  n.items.clear();
  n.items.shrink_to_fit();
}

void HbstPlaceRecognizer::insert(Node& n, std::uint32_t item, int depth) {
  Node* cur = &n;
  int d = depth;
  while (cur->bit >= 0) { cur = cur->child[bitOf(descs_[item], cur->bit) ? 1 : 0].get(); ++d; }
  cur->items.push_back(item);
  if (static_cast<int>(cur->items.size()) > opt_.max_leaf) split(*cur, d);
}

void HbstPlaceRecognizer::add(core::KeyframeId kf, core::MapId map, const std::vector<std::uint8_t>& descriptors, int stride) {
  if (stride <= 0) return;
  std::unique_lock lk(mu_);
  keyframes_.insert(kf);
  erased_.erase(kf);
  const std::size_t n = descriptors.size() / static_cast<std::size_t>(stride);
  for (std::size_t r = 0; r < n; ++r) {
    Descriptor d{};
    std::memcpy(d.data(), descriptors.data() + r * static_cast<std::size_t>(stride), static_cast<std::size_t>(std::min(stride, 32)));
    if (allZero(d)) continue;  // border keypoints without a descriptor
    const auto item = static_cast<std::uint32_t>(descs_.size());
    descs_.push_back(d);
    entries_.push_back({kf, map});
    insert(*root_, item, 0);
  }
}

void HbstPlaceRecognizer::erase(core::KeyframeId kf) {
  std::unique_lock lk(mu_);
  if (keyframes_.erase(kf)) erased_.insert(kf);  // lazy: entries stay in the tree, votes are dropped
}

std::vector<PlaceCandidate> HbstPlaceRecognizer::query(const std::vector<std::uint8_t>& descriptors, int stride,
                                                       const std::vector<core::KeyframeId>& exclude, int max_candidates) const {
  std::vector<PlaceCandidate> out;
  if (stride <= 0 || max_candidates <= 0) return out;
  std::shared_lock lk(mu_);
  if (descs_.empty()) return out;
  const std::unordered_set<core::KeyframeId> excluded(exclude.begin(), exclude.end());
  std::unordered_map<core::KeyframeId, int> votes;
  std::unordered_map<core::KeyframeId, core::MapId> maps;
  const std::size_t n = descriptors.size() / static_cast<std::size_t>(stride);
  int used = 0;
  for (std::size_t r = 0; r < n; ++r) {
    Descriptor q{};
    std::memcpy(q.data(), descriptors.data() + r * static_cast<std::size_t>(stride), static_cast<std::size_t>(std::min(stride, 32)));
    if (allZero(q)) continue;
    ++used;
    const Node* cur = root_.get();
    while (cur->bit >= 0) cur = cur->child[bitOf(q, cur->bit) ? 1 : 0].get();
    // Best admissible item of the leaf: erased and excluded keyframes (the query's own neighbourhood for loop
    // detection) do not take part, otherwise the query keyframe's own descriptors would always win.
    int best = opt_.max_hamming + 1;
    std::uint32_t best_item = 0;
    for (auto i : cur->items) {
      const auto kf = entries_[i].kf;
      if (erased_.count(kf) || excluded.count(kf)) continue;
      const int d = hamming(q, descs_[i]);
      if (d < best) { best = d; best_item = i; }
    }
    if (best > opt_.max_hamming) continue;
    const auto& e = entries_[best_item];
    ++votes[e.kf];
    maps[e.kf] = e.map;
  }
  for (const auto& [kf, v] : votes)
    if (v >= opt_.min_votes) out.push_back({kf, maps[kf], used > 0 ? static_cast<double>(v) / used : 0.0});
  std::sort(out.begin(), out.end(), [](const PlaceCandidate& a, const PlaceCandidate& b) {
    return a.score != b.score ? a.score > b.score : a.keyframe < b.keyframe;  // deterministic tie-break
  });
  if (static_cast<int>(out.size()) > max_candidates) out.resize(static_cast<std::size_t>(max_candidates));
  return out;
}

}  // namespace mvslam::map
