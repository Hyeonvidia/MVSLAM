#pragma once
// Worker = handler + LatestQueue + (Realtime) a std::jthread, or (Offline) inline execution from
// the caller thread. Exceptions inside the handler are logged and swallowed (host protection).
#include <exception>
#include <functional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>

#include "core/log.hpp"
#include "core/thread/latest_queue.hpp"

namespace mvslam::core {

enum class ExecutionPolicy { Async, Sync };

template <typename T>
class Worker {
 public:
  using Handler = std::function<void(T item, std::stop_token st)>;

  Worker(std::string name, ExecutionPolicy policy, Handler handler, typename LatestQueue<T>::MergeFn merge = nullptr)
      : name_(std::move(name)), policy_(policy), handler_(std::move(handler)), queue_(std::move(merge)) {
    if (policy_ == ExecutionPolicy::Async) {
      thread_ = std::jthread([this](std::stop_token st) { run(st); });
    }
  }
  ~Worker() { stop(); }
  Worker(const Worker&) = delete;
  Worker& operator=(const Worker&) = delete;

  // Async: enqueue (never blocks). Sync: run the handler now, on the caller thread, deterministically.
  void submit(T item) {
    if (policy_ == ExecutionPolicy::Async) {
      queue_.push(std::move(item));
    } else {
      invoke(std::move(item), std::stop_token{});
    }
  }
  void stop() {
    if (thread_.joinable()) {
      thread_.request_stop();
      thread_.join();
    }
  }
  bool busy() const { return busy_.load(std::memory_order_relaxed); }
  bool pending() const { return queue_.pending(); }
  const std::string& name() const { return name_; }

 private:
  void run(std::stop_token st) {
    while (auto item = queue_.pop(st)) invoke(std::move(*item), st);
  }
  void invoke(T item, std::stop_token st) {
    busy_.store(true, std::memory_order_relaxed);
    try {
      handler_(std::move(item), st);
    } catch (const std::exception& e) {
      MVSLAM_ERROR("worker {}: exception: {}", name_, e.what());
    } catch (...) {
      MVSLAM_ERROR("worker {}: unknown exception", name_);
    }
    busy_.store(false, std::memory_order_relaxed);
  }

  std::string name_;
  ExecutionPolicy policy_;
  Handler handler_;
  LatestQueue<T> queue_;
  std::atomic<bool> busy_{false};
  std::jthread thread_;  // last: joins first on destruction
};

}  // namespace mvslam::core
