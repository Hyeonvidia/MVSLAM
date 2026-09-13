#pragma once
// Online place recognition without a vocabulary: a Hamming binary search tree over the keyframes'
// ORB descriptors (Schlegel & Grisetti, HBST 2018). Each leaf holds up to `max_leaf` descriptors;
// a full leaf splits on its most balanced bit. A query descends by its own bits and brute-forces
// the leaf; every query descriptor casts one vote for the keyframe of its best match (<= max
// Hamming). Candidates are keyframes ranked by votes. Per-descriptor recall is approximate (a
// flipped split bit sends a descriptor to a sibling leaf), the image-level vote is robust.
// Thread-safe (shared_mutex): the mapper adds, the frontend queries.
#include <array>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_set>
#include <vector>

#include "map/place_recognizer.hpp"

namespace mvslam::map {

class HbstPlaceRecognizer final : public IPlaceRecognizer {
 public:
  struct Options { int max_leaf = 512; int max_depth = 24; int max_hamming = 50; int min_votes = 8; };
  HbstPlaceRecognizer() : HbstPlaceRecognizer(Options{}) {}
  explicit HbstPlaceRecognizer(Options o) : opt_(o), root_(std::make_unique<Node>()) {}
  void add(core::KeyframeId kf, core::MapId map, const std::vector<std::uint8_t>& descriptors, int stride) override;
  void erase(core::KeyframeId kf) override;
  std::vector<PlaceCandidate> query(const std::vector<std::uint8_t>& descriptors, int stride,
                                    const std::vector<core::KeyframeId>& exclude, int max_candidates) const override;
  std::size_t numDescriptors() const { std::shared_lock lk(mu_); return descs_.size(); }
  std::size_t numKeyframes() const { std::shared_lock lk(mu_); return keyframes_.size(); }

 private:
  using Descriptor = std::array<std::uint8_t, 32>;
  struct Node { int bit = -1; std::unique_ptr<Node> child[2]; std::vector<std::uint32_t> items; };
  struct Entry { core::KeyframeId kf; core::MapId map; };
  void insert(Node& n, std::uint32_t item, int depth);
  void split(Node& n, int depth);

  Options opt_;
  mutable std::shared_mutex mu_;
  std::vector<Descriptor> descs_;
  std::vector<Entry> entries_;
  std::unordered_set<core::KeyframeId> keyframes_, erased_;
  std::unique_ptr<Node> root_;
};

}  // namespace mvslam::map
