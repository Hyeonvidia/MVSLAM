#include <gtest/gtest.h>

#include <random>

#include "map/hbst_place_recognizer.hpp"

using namespace mvslam::core;

namespace {
std::vector<std::uint8_t> randomDescriptors(std::mt19937& rng, int n) {
  std::vector<std::uint8_t> d(static_cast<std::size_t>(n) * 32);
  std::uniform_int_distribution<int> b(0, 255);
  for (auto& x : d) x = static_cast<std::uint8_t>(b(rng));
  return d;
}
// Flips each bit with probability p (p = 0.1 -> ~26 of 256 bits, a typical re-observation distance).
std::vector<std::uint8_t> perturb(const std::vector<std::uint8_t>& d, double p, std::mt19937& rng) {
  std::vector<std::uint8_t> out = d;
  std::bernoulli_distribution flip(p);
  for (auto& byte : out) for (int b = 0; b < 8; ++b) if (flip(rng)) byte ^= static_cast<std::uint8_t>(1 << b);
  return out;
}
}  // namespace

TEST(HbstPlaceRecognizer, RecognizesPerturbedPlacesAndRejectsUnknownOnes) {
  std::mt19937 rng(21);
  mvslam::map::HbstPlaceRecognizer pr;
  const int places = 200, per_place = 300;
  std::vector<std::vector<std::uint8_t>> descs;
  for (int k = 1; k <= places; ++k) {
    descs.push_back(randomDescriptors(rng, per_place));
    pr.add(KeyframeId{static_cast<std::uint64_t>(k)}, MapId{1}, descs.back(), 32);
  }
  EXPECT_EQ(pr.numKeyframes(), 200u);
  EXPECT_EQ(pr.numDescriptors(), 60000u);

  int top1 = 0;
  for (int k = 1; k <= places; ++k) {
    // Query: 200 perturbed descriptors of place k + 100 random distractors.
    auto q = perturb(descs[static_cast<std::size_t>(k - 1)], 0.10, rng);
    q.resize(200 * 32);
    const auto extra = randomDescriptors(rng, 100);
    q.insert(q.end(), extra.begin(), extra.end());
    const auto c = pr.query(q, 32, {}, 3);
    if (!c.empty() && c.front().keyframe.value == static_cast<std::uint64_t>(k)) ++top1;
  }
  EXPECT_GE(top1, 190) << "top-1 hits: " << top1;  // >= 95% top-1

  int false_hits = 0;
  for (int i = 0; i < 50; ++i) if (!pr.query(randomDescriptors(rng, 300), 32, {}, 3).empty()) ++false_hits;
  EXPECT_LE(false_hits, 1);

  // Exclusion and erase drop candidates.
  const auto q = perturb(descs[4], 0.05, rng);
  const auto c5 = pr.query(q, 32, {}, 1);
  ASSERT_FALSE(c5.empty());
  EXPECT_EQ(c5.front().keyframe.value, 5u);
  for (const auto& c : pr.query(q, 32, {KeyframeId{5}}, 3)) EXPECT_NE(c.keyframe.value, 5u);
  pr.erase(KeyframeId{5});
  const auto after = pr.query(q, 32, {}, 3);
  for (const auto& c : after) EXPECT_NE(c.keyframe.value, 5u);
  EXPECT_EQ(pr.numKeyframes(), 199u);
}

TEST(HbstPlaceRecognizer, IgnoresZeroDescriptorsAndEmptyIndex) {
  mvslam::map::HbstPlaceRecognizer pr;
  EXPECT_TRUE(pr.query(std::vector<std::uint8_t>(64, 1), 32, {}, 5).empty());
  pr.add(KeyframeId{1}, MapId{1}, std::vector<std::uint8_t>(32, 0), 32);  // all-zero row is skipped
  EXPECT_EQ(pr.numDescriptors(), 0u);
}
