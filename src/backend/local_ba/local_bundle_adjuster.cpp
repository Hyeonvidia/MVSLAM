#include "backend/local_ba/local_bundle_adjuster.hpp"

#include <set>
#include <unordered_map>

#include "backend/graph/factors/reprojection_factor.hpp"
#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::backend {

namespace {
constexpr std::uint64_t kPointOffset = std::uint64_t{1} << 40;
core::VariableId poseVar(core::KeyframeId k) { return core::VariableId{k.value}; }
core::VariableId pointVar(core::MapPointId m) { return core::VariableId{m.value + kPointOffset}; }

struct ObsRef { core::KeyframeId kf; core::MapPointId mp; int slot; int cam; core::Vec2 uv; double info; graph::ReprojectionFactor* factor = nullptr; };
}  // namespace

LocalBundleAdjuster::LocalBundleAdjuster(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver)
    : cfg_(cfg), rig_(rig), solver_(std::move(solver)) {
  selector_ = std::make_unique<CovisibilityWindow>(cfg.backend);  // TemporalWindow for inertial: P2
}

LocalBundleAdjuster::Report LocalBundleAdjuster::run(map::Map& map, core::KeyframeId latest, std::stop_token st,
                                                     std::function<bool()> newer_keyframe_pending) {
  core::ScopedTimer timer("lba.run");
  Report rep;
  // ---- 1. snapshot under read lock -------------------------------------------------------
  BaWindow window;
  std::unordered_map<core::KeyframeId, core::SE3> poses;
  std::unordered_map<core::MapPointId, core::Vec3> points;
  std::vector<ObsRef> obs;
  std::vector<std::pair<core::KeyframeId, int>> outliers;  // pre-flagged (depth <= 0) + chi2 outliers after solving
  map.withReadLock([&](const map::Map& m) {
    window = selector_->select(m, latest);
    std::set<core::KeyframeId> in_window(window.optimized.begin(), window.optimized.end());
    std::set<core::KeyframeId> in_fixed(window.fixed.begin(), window.fixed.end());
    for (auto k : window.optimized) if (auto kf = m.keyframe(k)) poses[k] = kf->T_wb();
    for (auto k : window.fixed) if (auto kf = m.keyframe(k)) poses[k] = kf->T_wb();
    for (auto k : window.optimized) {
      auto kf = m.keyframe(k);
      if (!kf) continue;
      for (auto mpid : kf->mapPointSlots()) {
        if (!mpid.valid() || points.count(mpid)) continue;
        auto mp = m.mapPoint(mpid);
        if (!mp || mp->bad()) continue;
        points[mpid] = mp->position();
        for (const auto& [okf, slot] : mp->observations()) {
          if (!in_window.count(okf) && !in_fixed.count(okf)) continue;
          auto okf_ptr = m.keyframe(okf);
          if (!okf_ptr || slot.keypoint_index < 0) continue;
          const auto& kp = okf_ptr->keypoints()[static_cast<std::size_t>(slot.keypoint_index)];
          if (kp.outlier) continue;
          if ((rig_.T_cb(0) * (poses[okf].inverse() * points[mpid])).z() <= 1e-3) { outliers.emplace_back(okf, slot.keypoint_index); continue; }
          obs.push_back({okf, mpid, slot.keypoint_index, 0, kp.uv, kp.info});
          if (kp.has_right) obs.push_back({okf, mpid, slot.keypoint_index, 1, kp.uv_right, kp.info});
        }
      }
    }
    return 0;
  });
  rep.keyframes = static_cast<int>(window.optimized.size());
  rep.points = static_cast<int>(points.size());
  rep.observations = static_cast<int>(obs.size());
  if (window.optimized.empty() || points.size() < 10) { last_ = rep; return rep; }

  // ---- 2. problem ----------------------------------------------------------------------------
  graph::Problem problem;
  for (const auto& [k, T] : poses) problem.addVariable(poseVar(k), graph::Variable::Pose(T));
  for (auto k : window.fixed) if (poses.count(k)) problem.setFixed(poseVar(k));
  if (window.fixed.empty()) problem.setFixed(poseVar(*std::min_element(window.optimized.begin(), window.optimized.end())));  // gauge
  for (const auto& [id, X] : points) problem.addVariable(pointVar(id), graph::Variable::Point(X));
  const double chi2_mono = cfg_.backend.huber_mono_chi2;
  for (auto& o : obs) {
    auto f = std::make_unique<graph::ReprojectionFactor>(poseVar(o.kf), pointVar(o.mp), rig_.camera(o.cam), rig_.T_cb(o.cam), o.uv, o.info);
    f->loss = graph::RobustLoss::Huber(std::sqrt(chi2_mono));
    o.factor = f.get();
    problem.addFactor(std::move(f));
  }
  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::SparseSchur;
  so.max_iterations = cfg_.backend.lba_iters;
  so.max_time_s = cfg_.execution == ExecutionMode::Offline ? 0.0 : cfg_.backend.lba_max_time_s;
  so.num_threads = cfg_.backend.solver_threads;
  if (cfg_.execution != ExecutionMode::Offline) so.stop_predicate = newer_keyframe_pending;
  const double initial_cost = problem.totalRobustCost();
  auto r1 = solver_->solve(problem, so, st);
  rep.ran = true;
  rep.preempted = r1.preempted;
  rep.initial_chi2 = initial_cost;
  const double robust_after = problem.totalRobustCost();
  if (robust_after > initial_cost * 1.05) {  // diverged (robust cost went up): keep the map untouched
    MVSLAM_WARN("lba: discarded (robust cost {:.1f} -> {:.1f})", initial_cost, robust_after);
    last_ = rep; return rep;
  }

  // ---- 3. outliers + optional L2 refinement ---------------------------------------------------
  std::set<graph::IFactor*> bad;
  for (const auto& o : obs) {
    if (o.factor->chi2(problem.values()) > chi2_mono || !o.factor->depthPositive(problem.values())) {
      bad.insert(o.factor);
      if (o.cam == 0) outliers.emplace_back(o.kf, o.slot);
    }
  }
  if (cfg_.backend.l2_refine_after_robust && !r1.preempted) {
    auto& fs = problem.factors();
    fs.erase(std::remove_if(fs.begin(), fs.end(), [&](const std::unique_ptr<graph::IFactor>& f) { return bad.count(f.get()) != 0; }), fs.end());
    for (auto& f : fs) f->loss = graph::RobustLoss::None();
    so.max_iterations = std::max(2, cfg_.backend.lba_iters / 2);
    solver_->solve(problem, so, st);
  }
  rep.final_chi2 = problem.totalRobustCost();
  rep.outliers = static_cast<int>(outliers.size());

  // ---- 4. write-back ---------------------------------------------------------------------------
  map.withWriteLock([&](map::Map& m) {
    for (auto k : window.optimized) if (auto kf = m.keyframe(k)) if (!kf->bad()) kf->setT_wb(problem.variable(poseVar(k)).pose);
    for (const auto& [id, _] : points) if (auto mp = m.mapPoint(id)) if (!mp->bad()) mp->setPosition(problem.variable(pointVar(id)).vec);
    for (const auto& [k, slot] : outliers) {
      auto kf = m.keyframe(k);
      if (!kf) continue;
      auto& kp = kf->keypoints()[static_cast<std::size_t>(slot)];
      kp.outlier = true;
      const auto mpid = kf->mapPointSlots()[static_cast<std::size_t>(slot)];
      if (mpid.valid()) {
        m.removeObservation(mpid, k);
        if (auto mp = m.mapPoint(mpid)) if (mp->numObservations() < 2) m.removeMapPoint(mpid);
      }
    }
    return 0;
  });
  rep.applied = true;
  rep.time_s = r1.time_s;
  last_ = rep;
  MVSLAM_DEBUG("lba: kfs={} pts={} obs={} outliers={} chi2 {:.1f}->{:.1f} {}", rep.keyframes, rep.points, rep.observations, rep.outliers,
               rep.initial_chi2, rep.final_chi2, rep.preempted ? "(preempted)" : "");
  return rep;
}

}  // namespace mvslam::backend
