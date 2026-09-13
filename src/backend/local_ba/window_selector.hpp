#pragma once
// Local BA window strategies (docs/design/03-backend.md §2): covisibility with cap (visual),
// covisibility-score truncation (OV2SLAM), temporal chain (inertial). Implementation: P0/P2.
#include <vector>

#include "core/ids.hpp"
#include "map/map.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::backend {

struct BaWindow { std::vector<core::KeyframeId> optimized, fixed; };

class IWindowSelector {
 public:
  virtual ~IWindowSelector() = default;
  virtual BaWindow select(const map::Map& map, core::KeyframeId latest) const = 0;
};

class CovisibilityWindow final : public IWindowSelector {  // ORB-SLAM3 visual LBA + cap
 public:
  explicit CovisibilityWindow(const BackendConfig& c) : c_(c) {}
  BaWindow select(const map::Map& map, core::KeyframeId latest) const override;
 private:
  BackendConfig c_;
};

class TemporalWindow final : public IWindowSelector {  // ORB-SLAM3 inertial LBA (prevKf chain)
 public:
  TemporalWindow(const BackendConfig& c, bool large) : c_(c), large_(large) {}
  BaWindow select(const map::Map& map, core::KeyframeId latest) const override;
 private:
  BackendConfig c_;
  bool large_;
};

}  // namespace mvslam::backend
