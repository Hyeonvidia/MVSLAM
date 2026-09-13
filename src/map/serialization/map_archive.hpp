#pragma once
// Versioned binary map persistence (cereal). Format: docs/design/04-map.md §6. Implementation P1.
#include <string>

#include "map/map_store.hpp"
#include "mvslam/api/types.hpp"

namespace mvslam::map {
Result<void> saveMapStore(const MapStore& store, const std::string& path);
Result<MapSummary> loadMapStore(MapStore& store, const std::string& path);
}  // namespace mvslam::map
