#include "map/map.hpp"

#include <algorithm>

namespace mvslam::map {

std::shared_ptr<KeyFrame> Map::addKeyFrame(std::shared_ptr<KeyFrame> kf) {
  if (keyframes_.empty()) origins_.push_back(kf->id());
  auto& slot = keyframes_[kf->id()];
  slot = std::move(kf);
  return slot;
}

std::shared_ptr<MapPoint> Map::addMapPoint(std::shared_ptr<MapPoint> mp) {
  auto& slot = map_points_[mp->id()];
  slot = std::move(mp);
  return slot;
}

void Map::addObservation(core::MapPointId mpid, core::KeyframeId kfid, ObservationSlot slot) {
  auto mp = mapPoint(mpid);
  auto kf = keyframe(kfid);
  if (!mp || !kf) return;
  mp->addObservation(kfid, slot);
  auto& slots = kf->mapPointSlots();
  if (slots.size() < kf->keypoints().size()) slots.resize(kf->keypoints().size());
  if (slot.keypoint_index >= 0 && static_cast<std::size_t>(slot.keypoint_index) < slots.size())
    slots[static_cast<std::size_t>(slot.keypoint_index)] = mpid;
}

void Map::removeObservation(core::MapPointId mpid, core::KeyframeId kfid) {
  auto mp = mapPoint(mpid);
  auto kf = keyframe(kfid);
  if (!mp || !kf) return;
  auto it = mp->observations().find(kfid);
  if (it != mp->observations().end()) {
    auto& slots = kf->mapPointSlots();
    const int idx = it->second.keypoint_index;
    if (idx >= 0 && static_cast<std::size_t>(idx) < slots.size()) slots[static_cast<std::size_t>(idx)] = core::MapPointId{};
  }
  mp->removeObservation(kfid);
}

void Map::removeKeyFrame(core::KeyframeId kfid) {
  auto it = keyframes_.find(kfid);
  if (it == keyframes_.end()) return;
  auto kf = it->second;
  std::vector<core::MapPointId> starved;  // ORB-SLAM: a point left with < 2 observations is discarded with the keyframe
  for (auto mpid : kf->mapPointSlots()) {
    if (!mpid.valid()) continue;
    if (auto mp = mapPoint(mpid)) {
      mp->removeObservation(kfid);  // includes anchor migration
      if (mp->numObservations() < 2) starved.push_back(mpid);
    }
  }
  for (auto mpid : starved) removeMapPoint(mpid);
  // Re-parent children to this keyframe's parent (simple spanning-tree repair).
  for (auto child : kf->children()) {
    if (auto c = keyframe(child)) c->setParent(kf->parent());
    if (auto p = keyframe(kf->parent())) p->children().insert(child);
  }
  if (auto p = keyframe(kf->parent())) p->children().erase(kfid);
  if (auto prev = keyframe(kf->prevKf())) prev->setNextKf(kf->nextKf());
  if (auto next = keyframe(kf->nextKf())) next->setPrevKf(kf->prevKf());
  cov_.erase(kfid);
  kf->markBad(changeEpoch());
  kf_tombstones_.push_back(kf);
  keyframes_.erase(it);
}

void Map::removeMapPoint(core::MapPointId mpid) {
  auto it = map_points_.find(mpid);
  if (it == map_points_.end()) return;
  auto mp = it->second;
  for (const auto& [kfid, slot] : mp->observations()) {
    if (auto kf = keyframe(kfid)) {
      auto& slots = kf->mapPointSlots();
      if (slot.keypoint_index >= 0 && static_cast<std::size_t>(slot.keypoint_index) < slots.size())
        slots[static_cast<std::size_t>(slot.keypoint_index)] = core::MapPointId{};
    }
  }
  mp->markBad(changeEpoch());
  mp_tombstones_.push_back(mp);
  map_points_.erase(it);
}

void Map::updateCovisibility(core::KeyframeId kfid, int min_shared) {
  auto kf = keyframe(kfid);
  if (!kf) return;
  std::unordered_map<core::KeyframeId, int> counts;
  for (auto mpid : kf->mapPointSlots()) {
    if (!mpid.valid()) continue;
    auto mp = mapPoint(mpid);
    if (!mp) continue;
    for (const auto& [other, _] : mp->observations()) if (other != kfid) ++counts[other];
  }
  core::KeyframeId best;
  int best_w = 0;
  for (const auto& [other, w] : counts) {
    if (w >= min_shared) cov_.setWeight(kfid, other, w);
    if (w > best_w || (w == best_w && other < best)) { best_w = w; best = other; }  // tie -> older keyframe (deterministic)
  }
  if (!kf->parent().valid() && best.valid()) {  // first connection: attach to the spanning tree
    kf->setParent(best);
    if (auto p = keyframe(best)) p->children().insert(kfid);
  }
}

void Map::reclaim(std::uint64_t oldest_live_epoch) {
  auto pred_kf = [&](const std::shared_ptr<KeyFrame>& k) { return k->badEpoch() < oldest_live_epoch; };
  auto pred_mp = [&](const std::shared_ptr<MapPoint>& m) { return m->badEpoch() < oldest_live_epoch; };
  kf_tombstones_.erase(std::remove_if(kf_tombstones_.begin(), kf_tombstones_.end(), pred_kf), kf_tombstones_.end());
  mp_tombstones_.erase(std::remove_if(mp_tombstones_.begin(), mp_tombstones_.end(), pred_mp), mp_tombstones_.end());
}

void Map::applyScaledRotation(double s, const core::SO3& R_gw) {
  auto xform = [&](const core::SE3& T) { return core::SE3(R_gw * T.so3(), s * (R_gw * T.translation())); };
  for (auto& [_, kf] : keyframes_) {
    kf->setT_wb(xform(kf->T_wb()));
    kf->setVelocity(s * (R_gw * kf->velocity()));
  }
  for (auto& [_, mp] : map_points_) mp->setPosition(s * (R_gw * mp->position()));
  for (auto& [_, rec] : culled_) {
    rec.T_wc_at_cull = xform(rec.T_wc_at_cull);
    rec.T_rc = core::SE3(rec.T_rc.so3(), s * rec.T_rc.translation());
  }
}

void Map::applyTransform(const core::Sim3& S) {
  const double s = S.scale();
  const core::SO3 R = S.rxso3().so3();
  auto pose = [&](const core::SE3& T) { return core::SE3(R * T.so3(), S * T.translation()); };
  for (auto& [_, kf] : keyframes_) {
    kf->setT_wb(pose(kf->T_wb()));
    kf->setVelocity(s * (R * kf->velocity()));
  }
  for (auto& [_, mp] : map_points_) mp->setPosition(S * mp->position());
  for (auto& [_, rec] : culled_) {
    rec.T_wc_at_cull = pose(rec.T_wc_at_cull);
    rec.T_rc = core::SE3(rec.T_rc.so3(), s * rec.T_rc.translation());
  }
}

void Map::mergeMapPoints(core::MapPointId keep, core::MapPointId remove) {
  if (keep == remove) return;
  auto k = mapPoint(keep);
  auto r = mapPoint(remove);
  if (!k || !r) return;
  const auto obs = r->observations();  // copy: removeObservation mutates
  for (const auto& [kfid, slot] : obs) {
    auto kf = keyframe(kfid);
    if (!kf) continue;
    if (k->observations().count(kfid)) {  // keyframe already observes `keep`: drop the duplicate slot
      auto& slots = kf->mapPointSlots();
      if (slot.keypoint_index >= 0 && static_cast<std::size_t>(slot.keypoint_index) < slots.size()) slots[static_cast<std::size_t>(slot.keypoint_index)] = core::MapPointId{};
      continue;
    }
    k->addObservation(kfid, slot);
    auto& slots = kf->mapPointSlots();
    if (slot.keypoint_index >= 0 && static_cast<std::size_t>(slot.keypoint_index) < slots.size()) {
      slots[static_cast<std::size_t>(slot.keypoint_index)] = keep;
      kf->keypoints()[static_cast<std::size_t>(slot.keypoint_index)].track = core::TrackId{keep.value};
    }
  }
  r->markBad(changeEpoch());
  mp_tombstones_.push_back(r);
  map_points_.erase(remove);
  addTrackAlias(core::TrackId{remove.value}, keep);
}

void Map::absorb(Map& other) {
  for (auto& [id, kf] : other.keyframes_) keyframes_[id] = kf;
  for (auto& [id, mp] : other.map_points_) map_points_[id] = mp;
  for (const auto& [a, nb] : other.cov_.adjacency()) for (const auto& [b, w] : nb) cov_.setWeight(a, b, w);
  for (const auto& [t, mp] : other.track_aliases_) track_aliases_[t] = mp;
  for (const auto& [k, rec] : other.culled_) culled_[k] = rec;
  for (const auto& [t, obs] : other.pending2d_) pending2d_[t] = obs;
  origins_.insert(origins_.end(), other.origins_.begin(), other.origins_.end());
  if (!other.imu_state_.initialized && imu_state_.initialized) {}  // keep the mature inertial state of this map
  else if (other.imu_state_.initialized && !imu_state_.initialized) imu_state_ = other.imu_state_;
  other.keyframes_.clear();
  other.map_points_.clear();
  other.cov_ = CovisibilityGraph{};
  other.track_aliases_.clear();
  other.culled_.clear();
  other.pending2d_.clear();
  other.origins_.clear();
}

std::shared_ptr<KeyFrame> Map::keyframe(core::KeyframeId id) const {
  auto it = keyframes_.find(id);
  return it == keyframes_.end() ? nullptr : it->second;
}
std::shared_ptr<MapPoint> Map::mapPoint(core::MapPointId id) const {
  auto it = map_points_.find(id);
  return it == map_points_.end() ? nullptr : it->second;
}
std::vector<std::shared_ptr<KeyFrame>> Map::keyframes() const {
  std::vector<std::shared_ptr<KeyFrame>> v;
  v.reserve(keyframes_.size());
  for (const auto& [_, kf] : keyframes_) v.push_back(kf);
  std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a->id() < b->id(); });
  return v;
}
std::vector<std::shared_ptr<MapPoint>> Map::mapPoints() const {
  std::vector<std::shared_ptr<MapPoint>> v;
  v.reserve(map_points_.size());
  for (const auto& [_, mp] : map_points_) v.push_back(mp);
  return v;
}

}  // namespace mvslam::map
