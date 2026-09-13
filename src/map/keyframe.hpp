#pragma once
// KeyFrame: owned by Map (shared_ptr), referenced by id everywhere else (no pointer graph).
// Writes only under Map write lock. docs/design/04-map.md §1-2.
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "core/ids.hpp"
#include "core/math/se3.hpp"

namespace mvslam::map {

struct Keypoint {
  core::Vec2 uv_raw, uv;      // raw and undistorted pixel coordinates
  int level = 0;              // pyramid level (0 for KLT detections)
  int cam = 0;                // rig camera index
  core::TrackId track;        // KLT track id (== candidate MapPointId)
  double depth = -1;          // metric depth from stereo/rgbd at keyframe time, <0 if unknown
  double info = 1.0;          // observation weight (Sigma^-1 = info * I)
  bool has_right = false;     // stereo: matched undistorted pixel in camera 1
  core::Vec2 uv_right = core::Vec2::Zero();
  bool outlier = false;       // marked by BA; excluded from further optimization
};

struct ImuPreintegrationData;  // defined in backend/imu; stored by pointer to keep map free of backend types

class KeyFrame {
 public:
  KeyFrame(core::KeyframeId id, std::int64_t timestamp, core::SE3 T_wb) : id_(id), timestamp_(timestamp), T_wb_(std::move(T_wb)) {}
  core::KeyframeId id() const { return id_; }
  std::int64_t timestamp() const { return timestamp_; }
  const core::SE3& T_wb() const { return T_wb_; }
  void setT_wb(const core::SE3& T) { T_wb_ = T; }
  core::Vec3 velocity() const { return v_w_; }
  void setVelocity(const core::Vec3& v) { v_w_ = v; }
  const core::Vec6& bias() const { return bias_; }
  void setBias(const core::Vec6& b) { bias_ = b; }

  std::vector<Keypoint>& keypoints() { return keypoints_; }
  const std::vector<Keypoint>& keypoints() const { return keypoints_; }
  std::vector<std::uint8_t>& descriptors() { return descriptors_; }  // N x 32 bytes, row-major
  const std::vector<std::uint8_t>& descriptors() const { return descriptors_; }
  std::vector<core::MapPointId>& mapPointSlots() { return map_point_slots_; }  // index == keypoint index
  const std::vector<core::MapPointId>& mapPointSlots() const { return map_point_slots_; }

  core::KeyframeId parent() const { return parent_; }
  void setParent(core::KeyframeId p) { parent_ = p; }
  std::set<core::KeyframeId>& children() { return children_; }
  const std::set<core::KeyframeId>& children() const { return children_; }
  std::set<core::KeyframeId>& loopEdges() { return loop_edges_; }
  const std::set<core::KeyframeId>& loopEdges() const { return loop_edges_; }
  core::KeyframeId prevKf() const { return prev_; }
  core::KeyframeId nextKf() const { return next_; }
  void setPrevKf(core::KeyframeId k) { prev_ = k; }
  void setNextKf(core::KeyframeId k) { next_ = k; }

  bool bad() const { return bad_; }
  void markBad(std::uint64_t epoch) { bad_ = true; bad_epoch_ = epoch; }
  std::uint64_t badEpoch() const { return bad_epoch_; }

 private:
  core::KeyframeId id_;
  std::int64_t timestamp_;
  core::SE3 T_wb_;
  core::Vec3 v_w_ = core::Vec3::Zero();
  core::Vec6 bias_ = core::Vec6::Zero();
  std::vector<Keypoint> keypoints_;
  std::vector<std::uint8_t> descriptors_;
  std::vector<core::MapPointId> map_point_slots_;
  core::KeyframeId parent_, prev_, next_;
  std::set<core::KeyframeId> children_, loop_edges_;
  bool bad_ = false;
  std::uint64_t bad_epoch_ = 0;
};

}  // namespace mvslam::map
