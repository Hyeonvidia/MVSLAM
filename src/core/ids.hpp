#pragma once
// Strongly-typed 64-bit ids, globally monotonic across maps and sessions (docs/design/04-map.md §1).
#include <atomic>
#include <cstdint>
#include <functional>

namespace mvslam::core {

template <typename Tag>
struct Id {
  std::uint64_t value = 0;
  constexpr Id() = default;
  constexpr explicit Id(std::uint64_t v) : value(v) {}
  constexpr bool valid() const { return value != 0; }
  friend constexpr bool operator==(Id a, Id b) { return a.value == b.value; }
  friend constexpr bool operator!=(Id a, Id b) { return a.value != b.value; }
  friend constexpr bool operator<(Id a, Id b) { return a.value < b.value; }
};
struct KeyframeTag {};
struct MapPointTag {};
struct MapTag {};
struct TrackTag {};
struct VariableTag {};
using KeyframeId = Id<KeyframeTag>;
using MapPointId = Id<MapPointTag>;
using MapId = Id<MapTag>;
using TrackId = Id<TrackTag>;
using VariableId = Id<VariableTag>;

// Thread-safe monotonic generator. Value 0 is reserved as "invalid". Serialized with the map so a
// resumed session never reuses an id.
template <typename IdT>
class IdGenerator {
 public:
  explicit IdGenerator(std::uint64_t first = 1) : next_(first) {}
  IdT next() { return IdT{next_.fetch_add(1, std::memory_order_relaxed)}; }
  std::uint64_t peek() const { return next_.load(std::memory_order_relaxed); }
  void resume(std::uint64_t next) { next_.store(next, std::memory_order_relaxed); }
 private:
  std::atomic<std::uint64_t> next_;
};

}  // namespace mvslam::core

template <typename Tag>
struct std::hash<mvslam::core::Id<Tag>> {
  std::size_t operator()(mvslam::core::Id<Tag> id) const noexcept { return std::hash<std::uint64_t>{}(id.value); }
};
