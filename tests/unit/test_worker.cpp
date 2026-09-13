#include <gtest/gtest.h>

#include <atomic>
#include <chrono>

#include "core/thread/worker.hpp"

using namespace mvslam::core;

TEST(Worker, SyncRunsInline) {
  std::atomic<int> sum{0};
  Worker<int> w("sync", ExecutionPolicy::Sync, [&](int v, std::stop_token) { sum += v; });
  w.submit(1); w.submit(2); w.submit(3);
  EXPECT_EQ(sum.load(), 6);  // deterministic: all handled before submit returns
}

TEST(Worker, AsyncProcessesAndStopsCleanly) {
  std::atomic<int> handled{0};
  {
    Worker<int> w("async", ExecutionPolicy::Async, [&](int, std::stop_token) { handled++; });
    for (int i = 0; i < 50; ++i) w.submit(i);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (w.pending() && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }  // destructor joins
  EXPECT_GE(handled.load(), 1);
  EXPECT_LE(handled.load(), 50);
}

TEST(Worker, ExceptionsAreSwallowed) {
  Worker<int> w("throwing", ExecutionPolicy::Sync, [](int, std::stop_token) { throw std::runtime_error("boom"); });
  EXPECT_NO_THROW(w.submit(1));
}
