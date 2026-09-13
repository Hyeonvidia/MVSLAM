#include "backend/imu/inertial_optimization.hpp"

#include <Eigen/Dense>

#include "backend/graph/factors/imu_factor.hpp"
#include "backend/graph/problem.hpp"
#include "core/log.hpp"

namespace mvslam::backend::imu {

namespace {
constexpr std::uint64_t kVelOffset = std::uint64_t{1} << 40, kPoseOffset = std::uint64_t{1} << 41;
core::VariableId poseVar(std::size_t i) { return core::VariableId{kPoseOffset + i}; }
core::VariableId velVar(std::size_t i) { return core::VariableId{kVelOffset + i}; }
const core::VariableId kBias{1}, kGravity{2}, kScale{3};
}  // namespace

core::SO3 initialGravityDirection(const std::vector<KeyframeInertialState>& chain) {
  core::Vec3 dirG = core::Vec3::Zero();
  for (std::size_t i = 1; i < chain.size(); ++i) {
    if (!chain[i].preint) continue;
    dirG -= chain[i - 1].T_wb.rotationMatrix() * chain[i].preint->deltaV(chain[i].preint->bias_lin);
  }
  if (dirG.norm() < 1e-9) return core::SO3();
  dirG.normalize();
  const core::Vec3 gI(0, 0, -1);
  const core::Vec3 v = gI.cross(dirG);
  const double ang = std::acos(std::clamp(gI.dot(dirG), -1.0, 1.0));
  if (v.norm() < 1e-9) return ang < 1e-3 ? core::SO3() : core::SO3::exp(core::Vec3(M_PI, 0, 0));
  return core::SO3::exp(v.normalized() * ang);  // maps gI onto dirG
}

std::vector<core::Vec3> initialVelocities(const std::vector<KeyframeInertialState>& chain) {
  std::vector<core::Vec3> v(chain.size(), core::Vec3::Zero());
  for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
    const double dt = static_cast<double>(chain[i + 1].timestamp - chain[i].timestamp) * 1e-9;
    if (dt > 1e-6) v[i] = (chain[i + 1].T_wb.translation() - chain[i].T_wb.translation()) / dt;
  }
  if (chain.size() >= 2) v.back() = v[chain.size() - 2];
  return v;
}

InertialOnlyResult inertialOnlyOptimization(const std::vector<KeyframeInertialState>& chain, const core::SO3& R_wg0, double scale0,
                                            const core::Vec6& bias0, const InertialOnlyOptions& opt, graph::ISolver& solver) {
  InertialOnlyResult res;
  if (chain.size() < 2) return res;
  graph::Problem p;
  for (std::size_t i = 0; i < chain.size(); ++i) {
    p.addVariable(poseVar(i), graph::Variable::Pose(chain[i].T_wb));
    p.setFixed(poseVar(i));
    p.addVariable(velVar(i), graph::Variable::Velocity(chain[i].v_w));
    if (!opt.optimize_velocities) p.setFixed(velVar(i));
  }
  p.addVariable(kBias, graph::Variable::Bias(bias0));
  if (!opt.optimize_bias) p.setFixed(kBias);
  p.addVariable(kGravity, graph::Variable::GravityDir(R_wg0));
  if (!opt.optimize_gravity) p.setFixed(kGravity);
  p.addVariable(kScale, graph::Variable::LogScale(std::log(std::max(scale0, 1e-6))));
  if (!opt.optimize_scale) p.setFixed(kScale);
  for (std::size_t i = 1; i < chain.size(); ++i) {
    if (!chain[i].preint || chain[i].preint->dt <= 0) continue;
    p.addFactor(std::make_unique<graph::ImuScaleGravityFactor>(poseVar(i - 1), velVar(i - 1), kBias, poseVar(i), velVar(i), kGravity, kScale,
                                                               chain[i].preint, opt.gravity));
    ++res.factors;
  }
  if (res.factors == 0) return res;
  if (opt.optimize_bias && (opt.prior_gyro > 0 || opt.prior_acc > 0))
    p.addFactor(std::make_unique<graph::BiasPriorFactor>(kBias, bias0, opt.prior_gyro, opt.prior_acc));
  graph::SolveOptions so;
  so.linear_solver = graph::LinearSolverType::DenseQr;
  so.max_iterations = opt.iterations;
  so.num_threads = 1;
  res.initial_chi2 = p.totalChi2();
  const auto rep = solver.solve(p, so);
  res.final_chi2 = p.totalChi2();
  res.ok = std::isfinite(res.final_chi2);
  res.scale = p.variable(kScale).scale();
  res.R_wg = p.variable(kGravity).rot;
  res.bias = p.variable(kBias).vec;
  res.velocities.resize(chain.size());
  for (std::size_t i = 0; i < chain.size(); ++i) res.velocities[i] = p.variable(velVar(i)).vec;
  // Marginal variance of the log-scale from the dense normal equations over the free variables.
  if (opt.optimize_scale) {
    std::vector<core::VariableId> order;
    for (std::size_t i = 0; i < chain.size(); ++i) if (opt.optimize_velocities) order.push_back(velVar(i));
    if (opt.optimize_bias) order.push_back(kBias);
    if (opt.optimize_gravity) order.push_back(kGravity);
    order.push_back(kScale);
    const auto d = p.linearizeDense(p.values(), order);
    if (d.dim > 0) {
      const Eigen::MatrixXd cov = (d.H + 1e-9 * Eigen::MatrixXd::Identity(d.dim, d.dim)).ldlt().solve(Eigen::MatrixXd::Identity(d.dim, d.dim));
      const int off = d.offset.at(kScale);
      res.scale_variance = cov(off, off);
    }
  }
  MVSLAM_DEBUG("inertial-only: {} factors, chi2 {:.1f} -> {:.1f} ({} it), scale {:.4f} (var {:.2e}), bias g [{:.4f} {:.4f} {:.4f}] a [{:.3f} {:.3f} {:.3f}]",
               res.factors, res.initial_chi2, res.final_chi2, rep.iterations, res.scale, res.scale_variance, res.bias(0), res.bias(1), res.bias(2),
               res.bias(3), res.bias(4), res.bias(5));
  return res;
}

}  // namespace mvslam::backend::imu
