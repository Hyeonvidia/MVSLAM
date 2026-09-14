#include "backend/loop/pose_graph_optimizer.hpp"

#include <set>

#include "backend/graph/factors/relative_pose_factor.hpp"
#include "backend/graph/problem.hpp"
#include "core/log.hpp"
#include "core/profiler.hpp"

namespace mvslam::backend::loop {

namespace {
core::VariableId poseVar(core::KeyframeId k) { return core::VariableId{k.value}; }
}  // namespace

PgoReport PoseGraphOptimizer::optimize(map::Map& map, const std::unordered_map<core::KeyframeId, core::SE3>& reference_poses,
                                       const std::vector<core::KeyframeId>& fixed,
                                       const std::vector<std::pair<core::KeyframeId, core::KeyframeId>>& loop_edges, const PgoOptions& opt) {
  core::ScopedTimer timer("pgo.run");
  PgoReport rep;
  struct Edge { core::KeyframeId a, b; bool loop; };
  std::vector<Edge> edges;
  std::unordered_map<core::KeyframeId, core::SE3> current;
  std::unordered_map<core::MapPointId, core::KeyframeId> point_anchor;
  map.withReadLock([&](const map::Map& m) {
    std::set<std::pair<std::uint64_t, std::uint64_t>> seen;
    auto add = [&](core::KeyframeId a, core::KeyframeId b, bool loop) {
      if (!a.valid() || !b.valid() || a == b || !m.keyframe(a) || !m.keyframe(b)) return;
      const auto key = std::minmax(a.value, b.value);
      if (!seen.insert(key).second) return;
      edges.push_back({a, b, loop});
    };
    for (const auto& kf : m.keyframes()) if (!kf->bad()) current[kf->id()] = kf->T_wb();
    // Loop edges first so that they win over covisibility edges between the same pair.
    for (const auto& [a, b] : loop_edges) add(a, b, true);
    for (const auto& kf : m.keyframes()) if (!kf->bad()) for (auto l : kf->loopEdges()) add(kf->id(), l, true);
    for (const auto& kf : m.keyframes()) {
      if (kf->bad()) continue;
      if (kf->parent().valid()) add(kf->id(), kf->parent(), false);
      for (const auto& [other, w] : m.covisibility().neighbours(kf->id())) if (w >= opt.strong_weight) add(kf->id(), other, false);
      if (opt.inertial_chain && kf->prevKf().valid()) add(kf->id(), kf->prevKf(), false);
    }
    for (const auto& mp : m.mapPoints()) if (!mp->bad()) point_anchor[mp->id()] = mp->anchor();
    return 0;
  });
  if (current.size() < 2 || edges.empty()) return rep;
  auto poseOf = [&](core::KeyframeId k, bool use_reference) -> const core::SE3& {
    if (!use_reference) return current.at(k);
    auto it = reference_poses.find(k);
    return it == reference_poses.end() ? current.at(k) : it->second;
  };
  graph::Problem p;
  for (const auto& [k, T] : current) p.addVariable(poseVar(k), graph::Variable::Pose(T));
  for (auto f : fixed) if (current.count(f)) p.setFixed(poseVar(f));
  if (fixed.empty()) p.setFixed(poseVar(current.begin()->first));
  for (const auto& e : edges) {
    const core::SE3& Ta = poseOf(e.a, !e.loop);
    const core::SE3& Tb = poseOf(e.b, !e.loop);
    p.addFactor(std::make_unique<graph::RelativePoseSE3Factor>(poseVar(e.a), poseVar(e.b), Ta.inverse() * Tb, e.loop ? opt.loop_edge_info : 1.0));
  }
  rep.nodes = static_cast<int>(current.size());
  rep.edges = static_cast<int>(edges.size());
  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::SparseNormalCholesky;
  so.max_iterations = opt.iterations;
  so.num_threads = 1;
  rep.initial_chi2 = p.totalChi2();
  solver_->solve(p, so);
  rep.final_chi2 = p.totalChi2();
  if (!std::isfinite(rep.final_chi2)) return rep;
  // Write-back: keyframes get the optimized poses; points move with their anchor keyframe.
  map.withWriteLock([&](map::Map& m) {
    std::unordered_map<core::KeyframeId, core::SE3> delta;   // T_new * T_old^-1 per keyframe
    for (const auto& [k, T_old] : current) {
      auto kf = m.keyframe(k);
      if (!kf || kf->bad()) continue;
      const core::SE3 T_new = p.variable(poseVar(k)).pose;
      delta[k] = T_new * T_old.inverse();
      kf->setT_wb(T_new);
      kf->setVelocity(delta[k].so3() * kf->velocity());
      rep.corrections.emplace_back(k, std::make_pair(T_old, T_new));
    }
    for (const auto& [mpid, anchor] : point_anchor) {
      auto mp = m.mapPoint(mpid);
      auto d = delta.find(anchor);
      if (!mp || mp->bad() || d == delta.end()) continue;
      mp->setPosition(d->second * mp->position());
    }
    return 0;
  });
  rep.applied = true;
  MVSLAM_INFO("pgo: {} nodes, {} edges, chi2 {:.1f} -> {:.1f}", rep.nodes, rep.edges, rep.initial_chi2, rep.final_chi2);
  return rep;
}

}  // namespace mvslam::backend::loop
