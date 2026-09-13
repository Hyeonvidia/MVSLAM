#include "backend/pose_optimizer.hpp"

#include <cmath>

#include "backend/graph/factors/reprojection_factor.hpp"

namespace mvslam::backend {

frontend::PoseOptimizationResult MotionOnlyBa::optimize(frontend::Frame& frame, double max_time_ms) {
  frontend::PoseOptimizationResult res;
  outliers_.clear();
  const core::VariableId pose_id{1};
  graph::Problem p;
  p.addVariable(pose_id, graph::Variable::Pose(frame.T_wb));
  std::vector<std::pair<int, const graph::ReprojectionFactor*>> factors;  // track index -> factor
  std::uint64_t next = 2;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const auto& t = frame.tracks[i];
    if (!t.is3d) continue;
    const core::VariableId pid{next++};
    p.addVariable(pid, graph::Variable::Point(t.X_w));
    p.setFixed(pid);
    auto f = std::make_unique<graph::ReprojectionFactor>(pose_id, pid, rig_.camera(t.cam), rig_.T_cb(t.cam), t.uv, t.info > 0 ? t.info : 1.0);
    f->loss = graph::RobustLoss::Huber(std::sqrt(f_.pose_huber_chi2));
    factors.emplace_back(static_cast<int>(i), f.get());
    p.addFactor(std::move(f));
  }
  if (factors.size() < 5) { res.ok = false; return res; }

  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::DenseQr;
  so.max_iterations = f_.pose_max_iters;
  so.max_time_s = max_time_ms > 0 ? max_time_ms * 1e-3 : 0;
  so.num_threads = b_.solver_threads;
  solver_->solve(p, so);

  // Pass 2: drop outliers (chi2 / negative depth) and refine with plain least squares.
  int inliers = 0;
  for (auto& [idx, f] : factors) {
    const bool bad = f->chi2(p.values()) > f_.pose_huber_chi2 || !f->depthPositive(p.values());
    if (bad) { outliers_.push_back(idx); frame.tracks[static_cast<std::size_t>(idx)].is3d = false; }
    else ++inliers;
  }
  if (b_.l2_refine_after_robust && inliers >= 5) {
    auto& fs = p.factors();
    fs.erase(std::remove_if(fs.begin(), fs.end(), [&](const std::unique_ptr<graph::IFactor>& f) {
               return f->chi2(p.values()) > f_.pose_huber_chi2; }), fs.end());
    for (auto& f : fs) f->loss = graph::RobustLoss::None();
    solver_->solve(p, so);
  }
  frame.T_wb = p.variable(pose_id).pose;
  res.ok = inliers >= 15;
  res.inliers = inliers;
  res.chi2 = p.totalChi2();
  return res;
}

}  // namespace mvslam::backend
