#pragma once
// RAII scoped timer feeding a per-name running mean/max (cheap; used for SystemStats).
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>

namespace mvslam::core {

class Profiler {
 public:
  struct Stat { double mean_ms = 0, max_ms = 0; std::uint64_t count = 0; };
  static Profiler& instance() { static Profiler p; return p; }
  void add(const std::string& name, double ms) {
    std::lock_guard<std::mutex> lk(mu_);
    auto& s = stats_[name];
    s.count++;
    s.mean_ms += (ms - s.mean_ms) / static_cast<double>(s.count);
    if (ms > s.max_ms) s.max_ms = ms;
  }
  Stat get(const std::string& name) const {
    std::lock_guard<std::mutex> lk(mu_);
    auto it = stats_.find(name);
    return it == stats_.end() ? Stat{} : it->second;
  }
 private:
  mutable std::mutex mu_;
  std::unordered_map<std::string, Stat> stats_;
};

class ScopedTimer {
 public:
  explicit ScopedTimer(std::string name) : name_(std::move(name)), t0_(std::chrono::steady_clock::now()) {}
  ~ScopedTimer() {
    const auto dt = std::chrono::steady_clock::now() - t0_;
    Profiler::instance().add(name_, std::chrono::duration<double, std::milli>(dt).count());
  }
 private:
  std::string name_;
  std::chrono::steady_clock::time_point t0_;
};

}  // namespace mvslam::core
