#pragma once
// Place recognition strategy: candidate proposer only — geometric verification lives in
// backend/loop (LoopVerifier). Implementations: DBoW adapter (P3), incremental vocabulary (later).
#include <cstdint>
#include <vector>

#include "core/ids.hpp"

namespace mvslam::map {

struct PlaceCandidate { core::KeyframeId keyframe; core::MapId map; double score = 0; };

class IPlaceRecognizer {
 public:
  virtual ~IPlaceRecognizer() = default;
  virtual void add(core::KeyframeId kf, core::MapId map, const std::vector<std::uint8_t>& descriptors, int stride) = 0;
  virtual void erase(core::KeyframeId kf) = 0;
  // Candidates for loop/merge (exclude the query's covisible set) or relocalization (all maps).
  virtual std::vector<PlaceCandidate> query(const std::vector<std::uint8_t>& descriptors, int stride,
                                            const std::vector<core::KeyframeId>& exclude, int max_candidates) const = 0;
};

class NullPlaceRecognizer final : public IPlaceRecognizer {  // Null Object: loop closing disabled
 public:
  void add(core::KeyframeId, core::MapId, const std::vector<std::uint8_t>&, int) override {}
  void erase(core::KeyframeId) override {}
  std::vector<PlaceCandidate> query(const std::vector<std::uint8_t>&, int, const std::vector<core::KeyframeId>&, int) const override { return {}; }
};

}  // namespace mvslam::map
