#include <gtest/gtest.h>

#include <thread>
#include <unordered_set>

#include "core/ids.hpp"

using namespace mvslam::core;

TEST(Ids, MonotonicAndUniqueAcrossThreads) {
  IdGenerator<KeyframeId> gen;
  std::vector<std::vector<KeyframeId>> per(4);
  std::vector<std::thread> ts;
  for (int t = 0; t < 4; ++t) ts.emplace_back([&, t] { for (int i = 0; i < 1000; ++i) per[static_cast<std::size_t>(t)].push_back(gen.next()); });
  for (auto& t : ts) t.join();
  std::unordered_set<KeyframeId> all;
  for (auto& v : per) for (auto id : v) { EXPECT_TRUE(id.valid()); EXPECT_TRUE(all.insert(id).second); }
  EXPECT_EQ(all.size(), 4000u);
  EXPECT_EQ(gen.peek(), 4001u);
}

TEST(Ids, ResumeAfterLoad) {
  IdGenerator<MapPointId> gen;
  gen.resume(500);
  EXPECT_EQ(gen.next().value, 500u);
}
