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
