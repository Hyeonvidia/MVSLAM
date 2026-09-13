#include <gtest/gtest.h>

#include <thread>

#include "core/thread/latest_queue.hpp"

using namespace mvslam::core;

TEST(LatestQueue, KeepsOnlyLatestAndMerges) {
  struct Job { int id; std::vector<int> joined; };
  LatestQueue<Job> q([](const Job& dropped, Job& incoming) {
    incoming.joined.push_back(dropped.id);
    incoming.joined.insert(incoming.joined.end(), dropped.joined.begin(), dropped.joined.end());
  });
  EXPECT_FALSE(q.push({1, {}}));
  EXPECT_TRUE(q.push({2, {}}));
  EXPECT_TRUE(q.push({3, {}}));
  auto j = q.tryPop();
  ASSERT_TRUE(j);
  EXPECT_EQ(j->id, 3);
  EXPECT_EQ(j->joined, (std::vector<int>{2, 1}));
  EXPECT_EQ(q.droppedCount(), 2u);
  EXPECT_FALSE(q.tryPop());
}

TEST(LatestQueue, PopReturnsNulloptOnStop) {
  LatestQueue<int> q;
  std::stop_source src;
  std::thread t([&] { src.request_stop(); });
  auto r = q.pop(src.get_token());
  t.join();
  EXPECT_FALSE(r);
}
