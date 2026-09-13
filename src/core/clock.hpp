#pragma once
// Injected clock (docs/design/06-concurrency.md §5): Offline mode uses data time, never wall-clock.
#include <chrono>
#include <cstdint>

namespace mvslam::core {

class IClock {
 public:
  virtual ~IClock() = default;
  virtual std::int64_t nowNs() const = 0;
};

class SteadyClock final : public IClock {
 public:
  std::int64_t nowNs() const override {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
  }
};

class ManualClock final : public IClock {
 public:
  std::int64_t nowNs() const override { return now_; }
  void set(std::int64_t ns) { now_ = ns; }
 private:
  std::int64_t now_ = 0;
};

}  // namespace mvslam::core
