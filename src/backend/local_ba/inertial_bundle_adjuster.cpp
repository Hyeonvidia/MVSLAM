#include "backend/local_ba/inertial_bundle_adjuster.hpp"

#include <algorithm>
#include <set>
#include <unordered_map>

#include "backend/graph/factors/imu_factor.hpp"
#include "backend/graph/factors/reprojection_factor.hpp"
#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::backend {

namespace {
constexpr std::uint64_t kPoint = std::uint64_t{1} << 40, kVel = std::uint64_t{1} << 41, kBias = std::uint64_t{1} << 42;
core::VariableId poseVar(core::KeyframeId k) { return core::VariableId{k.value}; }
core::VariableId velVar(core::KeyframeId k) { return core::VariableId{k.value + kVel}; }
core::VariableId biasVar(core::KeyframeId k) { return core::VariableId{k.value + kBias}; }
core::VariableId pointVar(core::MapPointId m) { return core::VariableId{m.value + kPoint}; }
struct ObsRef { core::KeyframeId kf; core::MapPointId mp; int slot; int cam; core::Vec2 uv; double info; graph::ReprojectionFactor* factor = nullptr; };
struct KfState { core::KeyframeId id; core::SE3 T; core::Vec3 v; core::Vec6 b; std::shared_ptr<const sensors::imu::Preintegration> preint; };
}  // namespace

InertialBundleAdjuster::InertialBundleAdjuster(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver)
    : cfg_(cfg), rig_(rig), solver_(std::move(solver)) {}

InertialBundleAdjuster::Report InertialBundleAdjuster::run(map::Map& map, const std::vector<core::KeyframeId>& window, const InertialBaOptions& opt,
                                                           std::stop_token st, std::function<bool()> newer_keyframe_pending) {
  core::ScopedTimer timer("lba.run");
  Report rep;
  if (window.empty()) { last_ = rep; return rep; }
  // ---- 1. snapshot -------------------------------------------------------------------------------
  std::vector<KfState> chain;      // [boundary?] + window
  bool has_boundary = false;
  std::unordered_map<core::KeyframeId, core::SE3> fixed_poses;
  std::unordered_map<core::MapPointId, core::Vec3> points;
  std::vector<ObsRef> obs;
  std::vector<std::pair<core::KeyframeId, int>> outliers;
  map.withReadLock([&](const map::Map& m) {
    std::set<core::KeyframeId> in_window(window.begin(), window.end());
    auto first = m.keyframe(window.front());
    if (first && first->prevKf().valid()) {
      if (auto b = m.keyframe(first->prevKf()); b && !b->bad()) {
        chain.push_back({b->id(), b->T_wb(), b->velocity(), b->bias(), nullptr});
        has_boundary = true;
      }
    }
    for (auto k : window) {
      auto kf = m.keyframe(k);
      if (!kf || kf->bad()) continue;
      chain.push_back({k, kf->T_wb(), kf->velocity(), kf->bias(), kf->preintegration()});
    }
    int fixed_count = 0;
    for (auto k : window) {
      auto kf = m.keyframe(k);
      if (!kf) continue;
      for (auto mpid : kf->mapPointSlots()) {
        if (!mpid.valid() || points.count(mpid)) continue;
        auto mp = m.mapPoint(mpid);
        if (!mp || mp->bad()) continue;
        points[mpid] = mp->position();
        for (const auto& [okf, slot] : mp->observations()) {
          auto okf_ptr = m.keyframe(okf);
          if (!okf_ptr || slot.keypoint_index < 0) continue;
          if (!in_window.count(okf)) {
            if (!fixed_poses.count(okf)) {
              if (fixed_count >= opt.max_fixed_visual) continue;
              fixed_poses[okf] = okf_ptr->T_wb();
              ++fixed_count;
            }
          }
          const auto& kp = okf_ptr->keypoints()[static_cast<std::size_t>(slot.keypoint_index)];
          if (kp.outlier) continue;
          const core::SE3& T = in_window.count(okf) ? okf_ptr->T_wb() : fixed_poses[okf];
          if ((rig_.T_cb(0) * (T.inverse() * mp->position())).z() <= 1e-3) { outliers.emplace_back(okf, slot.keypoint_index); continue; }
          obs.push_back({okf, mpid, slot.keypoint_index, 0, kp.uv, kp.info});
          if (kp.has_right) obs.push_back({okf, mpid, slot.keypoint_index, 1, kp.uv_right, kp.info});
        }
      }
    }
    return 0;
  });
  rep.keyframes = static_cast<int>(window.size());
  rep.points = static_cast<int>(points.size());
  rep.observations = static_cast<int>(obs.size());
  if (chain.size() < 2) { last_ = rep; return rep; }

  // ---- 2. problem ----------------------------------------------------------------------------------
  graph::Problem problem;
  const std::size_t first_free = has_boundary ? 1 : 0;
  for (std::size_t i = 0; i < chain.size(); ++i) {
    const auto& s = chain[i];
    problem.addVariable(poseVar(s.id), graph::Variable::Pose(s.T));
    problem.addVariable(velVar(s.id), graph::Variable::Velocity(s.v));
    problem.addVariable(biasVar(s.id), graph::Variable::Bias(s.b));
    if (i < first_free) { problem.setFixed(poseVar(s.id)); problem.setFixed(velVar(s.id)); problem.setFixed(biasVar(s.id)); }
  }
  if (!has_boundary) problem.setFixed(poseVar(chain.front().id));  // gauge: the oldest window pose (velocity/bias stay free)
  for (const auto& [k, T] : fixed_poses) { problem.addVariable(poseVar(k), graph::Variable::Pose(T)); problem.setFixed(poseVar(k)); }
  for (const auto& [id, X] : points) problem.addVariable(pointVar(id), graph::Variable::Point(X));
  const double chi2_mono = cfg_.backend.huber_mono_chi2;
  for (auto& o : obs) {
    auto f = std::make_unique<graph::ReprojectionFactor>(poseVar(o.kf), pointVar(o.mp), rig_.camera(o.cam), rig_.T_cb(o.cam), o.uv, o.info);
    f->loss = graph::RobustLoss::Huber(std::sqrt(chi2_mono));
    o.factor = f.get();
    problem.addFactor(std::move(f));
  }
  const double G = rig_.hasImu() ? rig_.imu().gravity : 9.81;
  const core::Vec3 g_w(0, 0, -G);
  for (std::size_t i = 1; i < chain.size(); ++i) {
    const auto& a = chain[i - 1];
    const auto& b = chain[i];
    if (!b.preint || b.preint->dt <= 0) continue;
    // ORB-SLAM3 LocalInertialBA: the edge to the fixed boundary keyframe is down-weighted (x1e-2) and robustified
    // so an inconsistent boundary state does not bend the window; the other IMU edges keep the full information.
    const bool boundary_edge = has_boundary && i == 1;
    auto f = std::make_unique<graph::ImuFactor>(poseVar(a.id), velVar(a.id), biasVar(a.id), poseVar(b.id), velVar(b.id), b.preint, g_w,
                                                boundary_edge ? opt.boundary_info_scale : 1.0);
    if (boundary_edge || opt.robust_imu) f->loss = graph::RobustLoss::Huber(std::sqrt(16.92));
    problem.addFactor(std::move(f));
    problem.addFactor(std::make_unique<graph::BiasRandomWalkFactor>(biasVar(a.id), biasVar(b.id), b.preint));
    ++rep.imu_factors;
  }
  if (opt.bias_prior_gyro > 0 || opt.bias_prior_acc > 0)
    for (std::size_t i = first_free; i < chain.size(); ++i)
      problem.addFactor(std::make_unique<graph::BiasPriorFactor>(biasVar(chain[i].id), chain[i].b, opt.bias_prior_gyro, opt.bias_prior_acc));
  if (rep.imu_factors == 0 && points.size() < 10) { last_ = rep; return rep; }

  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::SparseSchur;
  so.max_iterations = opt.iterations;
  so.max_time_s = opt.max_time_s;
  so.num_threads = cfg_.backend.solver_threads;
  if (cfg_.execution != ExecutionMode::Offline) so.stop_predicate = newer_keyframe_pending;
  const double initial_cost = problem.totalRobustCost();
  auto r1 = solver_->solve(problem, so, st);
  rep.ran = true;
  rep.preempted = r1.preempted;
  rep.initial_cost = initial_cost;
  const double after = problem.totalRobustCost();
  if (!std::isfinite(after) || after > initial_cost * 1.05) {
    MVSLAM_WARN("inertial lba: discarded (robust cost {:.1f} -> {:.1f})", initial_cost, after);
    last_ = rep; return rep;
  }
  // ---- 3. visual outliers + L2 refinement --------------------------------------------------------
  std::set<graph::IFactor*> bad;
  for (const auto& o : obs) {
    if (o.factor->chi2(problem.values()) > chi2_mono || !o.factor->depthPositive(problem.values())) {
      bad.insert(o.factor);
      if (o.cam == 0) outliers.emplace_back(o.kf, o.slot);
    }
  }
  if (cfg_.backend.l2_refine_after_robust && !r1.preempted && !bad.empty()) {
    auto& fs = problem.factors();
    fs.erase(std::remove_if(fs.begin(), fs.end(), [&](const std::unique_ptr<graph::IFactor>& f) { return bad.count(f.get()) != 0; }), fs.end());
    for (auto& f : fs) f->loss = graph::RobustLoss::None();
    so.max_iterations = std::max(2, opt.iterations / 2);
    solver_->solve(problem, so, st);
  }
  rep.final_cost = problem.totalRobustCost();
  rep.outliers = static_cast<int>(outliers.size());
  double imu_chi2 = 0, rw_chi2 = 0; int n_imu = 0, n_rw = 0;
  for (const auto& f : problem.factors()) {
    if (f->name() == "Imu") { imu_chi2 += f->chi2(problem.values()); ++n_imu; }
    else if (f->name() == "BiasRandomWalk") { rw_chi2 += f->chi2(problem.values()); ++n_rw; }
  }
  MVSLAM_DEBUG("inertial lba residuals: imu chi2/factor {:.1f} (9 dof), bias-rw chi2/factor {:.1f} (6 dof)", n_imu ? imu_chi2 / n_imu : 0.0, n_rw ? rw_chi2 / n_rw : 0.0);
  // ---- 4. write-back -------------------------------------------------------------------------------
  map.withWriteLock([&](map::Map& m) {
    for (std::size_t i = first_free; i < chain.size(); ++i) {
      auto kf = m.keyframe(chain[i].id);
      if (!kf || kf->bad()) continue;
      kf->setT_wb(problem.variable(poseVar(chain[i].id)).pose);
      kf->setVelocity(problem.variable(velVar(chain[i].id)).vec);
      kf->setBias(problem.variable(biasVar(chain[i].id)).vec);
    }
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
  last_ = rep;
  MVSLAM_DEBUG("inertial lba: kfs={} imu={} pts={} obs={} outliers={} cost {:.1f}->{:.1f}{}", rep.keyframes, rep.imu_factors, rep.points,
               rep.observations, rep.outliers, rep.initial_cost, rep.final_cost, rep.preempted ? " (preempted)" : "");
  return rep;
}

}  // namespace mvslam::backend
