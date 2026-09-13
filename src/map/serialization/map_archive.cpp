#include "map/serialization/map_archive.hpp"

namespace mvslam::map {
Result<void> saveMapStore(const MapStore&, const std::string&) { return Result<void>::failure("map serialization: not implemented (P1)"); }
Result<MapSummary> loadMapStore(MapStore&, const std::string&) { return Result<MapSummary>::failure("map serialization: not implemented (P1)"); }
}  // namespace mvslam::map
