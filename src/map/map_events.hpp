#pragma once
// Observer: publish map events to subscribers outside locks; zero cost without subscribers.
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "mvslam/api/map_service.hpp"

namespace mvslam::map {

class MapEventBus {
 public:
  Subscription subscribe(EventHandler h) {
    std::lock_guard<std::mutex> lk(mu_);
    const auto id = next_id_++;
    handlers_[id] = std::move(h);
    auto self = weak_self_.lock();
    return Subscription([self, id] { if (self) self->unsubscribe(id); });
  }
  bool hasSubscribers() const { std::lock_guard<std::mutex> lk(mu_); return !handlers_.empty(); }
  void publish(const MapEvent& e) const {
    std::vector<EventHandler> copy;
    { std::lock_guard<std::mutex> lk(mu_); copy.reserve(handlers_.size()); for (const auto& [_, h] : handlers_) copy.push_back(h); }
    for (const auto& h : copy) { try { h(e); } catch (...) { /* subscriber errors never propagate */ } }
  }
  static std::shared_ptr<MapEventBus> create() { auto b = std::shared_ptr<MapEventBus>(new MapEventBus()); b->weak_self_ = b; return b; }
 private:
  MapEventBus() = default;
  void unsubscribe(std::uint64_t id) { std::lock_guard<std::mutex> lk(mu_); handlers_.erase(id); }
  mutable std::mutex mu_;
  std::unordered_map<std::uint64_t, EventHandler> handlers_;
  std::uint64_t next_id_ = 1;
  std::weak_ptr<MapEventBus> weak_self_;
};

}  // namespace mvslam::map
