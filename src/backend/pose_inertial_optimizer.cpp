#include "backend/pose_inertial_optimizer.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <set>

#include "backend/graph/factors/imu_factor.hpp"
#include "backend/graph/factors/reprojection_factor.hpp"
#include "backend/graph/problem.hpp"
#include "core/log.hpp"

namespace mvslam::backend {

namespace {
const core::VariableId kPoseI{1}, kVelI{2}, kBiasI{3}, kPoseJ{4}, kVelJ{5}, kBiasJ{6};

// Gaussian prior on the frame keys from the dense normal equations of `p` (all factors, at `p`'s values):
// [last-frame keys] are Schur-eliminated when present. Returns keys renamed to the "last frame" ids so the
// next problem can consume it directly.
std::unique_ptr<graph::PriorFactor> priorForNextFrame(const graph::Problem& p, bool has_last) {
  std::vector<core::VariableId> order;
  if (has_last) order = {kPoseI, kVelI, kBiasI, kPoseJ, kVelJ, kBiasJ};
  else order = {kPoseJ, kVelJ, kBiasJ};
  const auto d = p.linearizeDense(p.values(), order);
  Eigen::MatrixXd H = d.H;
  Eigen::VectorXd g = d.g;
  if (has_last) {  // Schur complement: eliminate the first 15 dimensions
    const int m = 15, n = d.dim - m;
    const Eigen::MatrixXd Hmm = H.topLeftCorner(m, m), Hmr = H.topRightCorner(m, n), Hrr = H.bottomRightCorner(n, n);
    const Eigen::VectorXd gm = g.head(m), gr = g.tail(n);
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (Hmm + Hmm.transpose()));
    Eigen::VectorXd inv = es.eigenvalues();
    for (int i = 0; i < inv.size(); ++i) inv(i) = inv(i) > 1e-8 ? 1.0 / inv(i) : 0.0;
    const Eigen::MatrixXd Hmm_inv = es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose();
    H = Hrr - Hmr.transpose() * Hmm_inv * Hmr;
    g = gr - Hmr.transpose() * Hmm_inv * gm;
  }
  graph::Values x0;
  x0[kPoseI] = p.variable(kPoseJ);
  x0[kVelI] = p.variable(kVelJ);
  x0[kBiasI] = p.variable(kBiasJ);
  return std::make_unique<graph::PriorFactor>(std::vector<core::VariableId>{kPoseI, kVelI, kBiasI}, std::move(x0), H, g);
}
}  // namespace

frontend::PoseOptimizationResult PoseInertialOptimizer::optimize(frontend::Frame& frame, double max_time_ms) {
  const auto* a = anchor_;
  if (!a || !a->valid || !a->preint || a->preint->dt <= 0) { prior_.reset(); return visual_.optimize(frame, max_time_ms); }
  const bool case_b = prior_ && a->last_ok && !a->map_updated && a->preint_last_frame && a->preint_last_frame->dt > 0;
  frontend::PoseOptimizationResult res;
  graph::Problem p;
  const double G = rig_.hasImu() ? rig_.imu().gravity : 9.81;
  const core::Vec3 g_w(0, 0, -G);
  if (case_b) {  // last frame is a variable held by its prior; IMU spans frame -> frame
    p.addVariable(kPoseI, graph::Variable::Pose(a->T_wb_last));
    p.addVariable(kVelI, graph::Variable::Velocity(a->v_last));
    p.addVariable(kBiasI, graph::Variable::Bias(a->bias_last));
    p.addFactor(std::make_unique<graph::PriorFactor>(std::vector<core::VariableId>{kPoseI, kVelI, kBiasI}, prior_->linearizationPoint(), prior_->H(), prior_->g()));
    p.addFactor(std::make_unique<graph::ImuFactor>(kPoseI, kVelI, kBiasI, kPoseJ, kVelJ, a->preint_last_frame, g_w));
    p.addFactor(std::make_unique<graph::BiasRandomWalkFactor>(kBiasI, kBiasJ, a->preint_last_frame));
  } else {       // last keyframe constant; IMU spans keyframe -> frame
    p.addVariable(kPoseI, graph::Variable::Pose(a->T_wb)); p.setFixed(kPoseI);
    p.addVariable(kVelI, graph::Variable::Velocity(a->v_w)); p.setFixed(kVelI);
    p.addVariable(kBiasI, graph::Variable::Bias(a->bias)); p.setFixed(kBiasI);
    p.addFactor(std::make_unique<graph::ImuFactor>(kPoseI, kVelI, kBiasI, kPoseJ, kVelJ, a->preint, g_w));
    p.addFactor(std::make_unique<graph::BiasRandomWalkFactor>(kBiasI, kBiasJ, a->preint));
  }
  p.addVariable(kPoseJ, graph::Variable::Pose(frame.T_wb));
  p.addVariable(kVelJ, graph::Variable::Velocity(frame.v_w));
  p.addVariable(kBiasJ, graph::Variable::Bias(frame.bias));
  std::vector<std::pair<int, const graph::ReprojectionFactor*>> visual;
  std::uint64_t next = 100;
  for (std::size_t i = 0; i < frame.tracks.size(); ++i) {
    const auto& t = frame.tracks[i];
    if (!t.is3d) continue;
    if ((rig_.T_cb(t.cam) * (frame.T_wb.inverse() * t.X_w)).z() <= 1e-3) { frame.tracks[i].is3d = false; continue; }
    const core::VariableId pid{next++};
    p.addVariable(pid, graph::Variable::Point(t.X_w));
    p.setFixed(pid);
    auto f = std::make_unique<graph::ReprojectionFactor>(kPoseJ, pid, rig_.camera(t.cam), rig_.T_cb(t.cam), t.uv, t.info > 0 ? t.info : 1.0);
    f->loss = graph::RobustLoss::Huber(std::sqrt(f_.pose_huber_chi2));
    visual.emplace_back(static_cast<int>(i), f.get());
    p.addFactor(std::move(f));
  }
  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::DenseQr;
  so.max_iterations = std::max(f_.pose_max_iters, 8);
  so.max_time_s = max_time_ms > 0 ? max_time_ms * 1e-3 : 0;
  so.num_threads = b_.solver_threads;
  solver_->solve(p, so);
  int inliers = 0;
  std::set<graph::IFactor*> bad;
  for (auto& [idx, f] : visual) {
    if (f->chi2(p.values()) > f_.pose_huber_chi2 || !f->depthPositive(p.values())) { frame.tracks[static_cast<std::size_t>(idx)].is3d = false; bad.insert(const_cast<graph::ReprojectionFactor*>(f)); }
    else ++inliers;
  }
  if (b_.l2_refine_after_robust && inliers >= 5) {
    auto& fs = p.factors();
    fs.erase(std::remove_if(fs.begin(), fs.end(), [&](const std::unique_ptr<graph::IFactor>& f) { return bad.count(f.get()) != 0; }), fs.end());
    for (auto& f : fs) if (f->name() == "Reprojection") f->loss = graph::RobustLoss::None();
    solver_->solve(p, so);
  }
  frame.T_wb = p.variable(kPoseJ).pose;
  frame.v_w = p.variable(kVelJ).vec;
  frame.bias = p.variable(kBiasJ).vec;
  res.ok = inliers >= 15;
  res.inliers = inliers;
  res.chi2 = p.totalChi2();
  if (res.ok) prior_ = priorForNextFrame(p, case_b);   // this frame's information for the next one
  else prior_.reset();
  return res;
}

}  // namespace mvslam::backend
