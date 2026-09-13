#include "backend/mapper/keyframe_culler.hpp"

#include <set>

#include "sensors/imu/preintegration.hpp"

namespace mvslam::backend {

std::vector<core::KeyframeId> KeyframeCuller::cull(map::Map& m, core::KeyframeId latest) const {
  std::vector<core::KeyframeId> removed;
  auto kf = m.keyframe(latest);
  if (!kf) return removed;
  const double redundancy = (cfg_.sensor == SensorMode::Stereo && cfg_.inertial == InertialMode::Imu)
                                ? cfg_.backend.kf_cull_redundancy_stereo_inertial : cfg_.backend.kf_cull_redundancy;
  const std::set<core::KeyframeId> origins(m.origins().begin(), m.origins().end());
  // Protect the two most recent keyframes (frontend references) and the map origin.
  const core::KeyframeId prev = kf->prevKf();
  for (const auto& [cand, weight] : m.covisibility().covisibles(latest, cfg_.backend.lba_min_covisibility)) {
    if (cand.value >= prev.value || origins.count(cand)) continue;  // newest keyframes are frontend references
    auto ckf = m.keyframe(cand);
    if (!ckf || ckf->bad()) continue;
    int total = 0, redundant = 0;
    for (auto mpid : ckf->mapPointSlots()) {
      if (!mpid.valid()) continue;
      auto mp = m.mapPoint(mpid);
      if (!mp || mp->bad()) continue;
      ++total;
      if (mp->numObservations() - 1 >= cfg_.backend.kf_cull_min_other_observers) ++redundant;
    }
    if (total >= 20 && redundant > redundancy * total) {
      // Inertial maps: the successor's IMU integral must absorb the culled interval (ORB-SLAM3 MergePrevious),
      // and the merged gap stays short (3 s after the second refinement, 0.5 s before it).
      const bool inertial = rig_ && rig_->hasImu() && m.imuState().initialized;
      if (inertial) {
        auto prev_kf = m.keyframe(ckf->prevKf());
        auto next_kf = m.keyframe(ckf->nextKf());
        if (!prev_kf || !next_kf || !ckf->preintegration() || !next_kf->preintegration()) continue;
        const double gap = static_cast<double>(next_kf->timestamp() - prev_kf->timestamp()) * 1e-9;
        if (gap > (m.imuState().refined2 ? 3.0 : 0.5)) continue;
        sensors::imu::Preintegrator merged(rig_->imu());
        merged.reset(next_kf->preintegration()->bias_lin);
        for (const auto& mm : next_kf->preintegration()->measurements) merged.integrate(mm.acc, mm.gyro, mm.dt);
        merged.mergePrevious(*ckf->preintegration());
        next_kf->setPreintegration(merged.snapshot());
      }
      map::Map::CulledRecord rec;
      rec.T_wc_at_cull = ckf->T_wb();
      for (auto ref : {ckf->parent(), ckf->prevKf(), ckf->nextKf()}) {  // first live reference wins (parent is oldest)
        auto rkf = ref.valid() ? m.keyframe(ref) : nullptr;
        if (!rkf || rkf->bad()) continue;
        rec.reference = ref;
        rec.T_rc = rkf->T_wb().inverse() * ckf->T_wb();
        break;
      }
      m.rememberCulled(cand, std::move(rec));
      m.removeKeyFrame(cand);
      removed.push_back(cand);
    }
  }
  return removed;
}

}  // namespace mvslam::backend
