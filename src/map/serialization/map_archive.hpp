#pragma once
// Versioned binary map persistence (cereal PortableBinary: little-endian, fixed-width; platform
// independent). Format: docs/design/04-map.md §6. Every container is written in id order so that
// save -> load -> save is byte-identical (round-trip test).
#include <string>

#include "map/map_store.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam::map {

// Writes every map holding at least one keyframe (stored and active, sorted by id) plus the
// keyframe / map point / track id counters. Tombstones, pending 2-D observations and the keyframe
// database are not written (the recognizer is rebuilt on load).
Result<void> saveMapStore(const MapStore& store, const std::string& path);
// Adds the archived maps to `store` as stored maps, resumes the id counters, re-creates an empty
// active map with a fresh id when the current one is empty, and re-registers every keyframe's
// descriptors with the store's place recognizer. Returns the summary of the loaded maps.
Result<MapSummary> loadMapStore(MapStore& store, const std::string& path);

}  // namespace mvslam::map
