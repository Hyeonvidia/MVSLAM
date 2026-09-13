#pragma once
// MapPoint: XYZ is canonical; anchored inverse depth is a BA-time parameterization only.
// Observations are keyed by KeyframeId (no pointers). docs/design/04-map.md §1-2.
#include <cstdint>
#include <map>
#include <vector>

#include "core/ids.hpp"
#include "core/math/se3.hpp"

namespace mvslam::map {

struct ObservationSlot { int keypoint_index = -1; int cam = 0; };

class MapPoint {
 public:
  MapPoint(core::MapPointId id, core::Vec3 X_w, core::KeyframeId anchor) : id_(id), X_w_(std::move(X_w)), anchor_(anchor) {}
  core::MapPointId id() const { return id_; }
  const core::Vec3& position() const { return X_w_; }
  void setPosition(const core::Vec3& X) { X_w_ = X; }
  core::KeyframeId anchor() const { return anchor_; }
  const std::map<core::KeyframeId, ObservationSlot>& observations() const { return obs_; }
  void addObservation(core::KeyframeId kf, ObservationSlot slot) { obs_[kf] = slot; }
  void removeObservation(core::KeyframeId kf) {
    obs_.erase(kf);
    if (kf == anchor_) anchor_ = obs_.empty() ? core::KeyframeId{} : obs_.begin()->first;  // oldest remaining observer
  }
  int numObservations() const { return static_cast<int>(obs_.size()); }
  const std::vector<std::uint8_t>& descriptor() const { return descriptor_; }
  void setDescriptor(std::vector<std::uint8_t> d) { descriptor_ = std::move(d); }
  void countVisible() { ++visible_; }
  void countFound() { ++found_; }
  double foundRatio() const { return visible_ == 0 ? 1.0 : static_cast<double>(found_) / static_cast<double>(visible_); }
  bool bad() const { return bad_; }
  void markBad(std::uint64_t epoch) { bad_ = true; bad_epoch_ = epoch; }
  std::uint64_t badEpoch() const { return bad_epoch_; }

 private:
  core::MapPointId id_;
  core::Vec3 X_w_;
  core::KeyframeId anchor_;
  std::map<core::KeyframeId, ObservationSlot> obs_;
  std::vector<std::uint8_t> descriptor_;
  int visible_ = 0, found_ = 0;
  bool bad_ = false;
  std::uint64_t bad_epoch_ = 0;
};

}  // namespace mvslam::map
