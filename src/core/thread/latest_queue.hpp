#pragma once
// Capacity-1 queue: the producer never blocks; a newer item replaces the pending one and the
// merge hook lets the consumer keep what matters from the dropped item (e.g. keyframe ids that
// must still join the covisibility window). docs/design/06-concurrency.md §2.
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>

namespace mvslam::core {

template <typename T>
class LatestQueue {
 public:
  using MergeFn = std::function<void(const T& dropped, T& incoming)>;
  explicit LatestQueue(MergeFn merge = nullptr) : merge_(std::move(merge)) {}

  // Returns true when a pending item was replaced (dropped).
  bool push(T item) {
    bool dropped = false;
    {
      std::lock_guard<std::mutex> lk(mu_);
      if (slot_) {
        if (merge_) merge_(*slot_, item);
        dropped = true;
        ++dropped_count_;
      }
      slot_ = std::move(item);
    }
    cv_.notify_one();
    return dropped;
  }

  // Blocks until an item is available or stop is requested; nullopt on stop.
  std::optional<T> pop(std::stop_token st) {
    std::unique_lock<std::mutex> lk(mu_);
    if (!cv_.wait(lk, st, [&] { return slot_.has_value(); })) return std::nullopt;
    std::optional<T> out = std::move(slot_);
    slot_.reset();
    return out;
  }

  std::optional<T> tryPop() {
    std::lock_guard<std::mutex> lk(mu_);
    std::optional<T> out = std::move(slot_);
    slot_.reset();
    return out;
  }

  bool pending() const { std::lock_guard<std::mutex> lk(mu_); return slot_.has_value(); }
  std::uint64_t droppedCount() const { std::lock_guard<std::mutex> lk(mu_); return dropped_count_; }

 private:
  mutable std::mutex mu_;
  std::condition_variable_any cv_;
  std::optional<T> slot_;
  MergeFn merge_;
  std::uint64_t dropped_count_ = 0;
};

}  // namespace mvslam::core
