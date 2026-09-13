#include "backend/solvers/ceres/ceres_solver.hpp"

#include <ceres/ceres.h>

#include <chrono>
#include <unordered_map>

namespace mvslam::backend::graph {
namespace {

int ambientSize(const Variable& v) {
  switch (v.kind) {
    case VariableKind::Pose: return 7;        // [t(3), q(x,y,z,w)]
    case VariableKind::GravityDir: return 4;  // q(x,y,z,w)
    default: return v.dim();
  }
}
void toAmbient(const Variable& v, double* x) {
  switch (v.kind) {
    case VariableKind::Pose: {
      const auto t = v.pose.translation(); const auto q = v.pose.unit_quaternion();
      x[0] = t.x(); x[1] = t.y(); x[2] = t.z(); x[3] = q.x(); x[4] = q.y(); x[5] = q.z(); x[6] = q.w();
      break;
    }
    case VariableKind::GravityDir: { const auto q = v.rot.unit_quaternion(); x[0] = q.x(); x[1] = q.y(); x[2] = q.z(); x[3] = q.w(); break; }
    default: for (int i = 0; i < v.dim(); ++i) x[i] = v.vec(i);
  }
}
void fromAmbient(const double* x, Variable& v) {
  switch (v.kind) {
    case VariableKind::Pose:
      v.pose = core::SE3(Eigen::Quaterniond(x[6], x[3], x[4], x[5]).normalized(), core::Vec3(x[0], x[1], x[2]));
      break;
    case VariableKind::GravityDir: v.rot = core::SO3(Eigen::Quaterniond(x[3], x[0], x[1], x[2]).normalized()); break;
    default: v.vec.resize(v.dim()); for (int i = 0; i < v.dim(); ++i) v.vec(i) = x[i];
  }
}

// Right-multiplicative SE3: x ⊞ delta = T(x) * exp(delta); PlusJacobian = [I6; 0] so that the cost
// function's ambient Jacobian [J_tangent | 0] composes to J_tangent.
class Se3Manifold final : public ceres::Manifold {
 public:
  int AmbientSize() const override { return 7; }
  int TangentSize() const override { return 6; }
  bool Plus(const double* x, const double* delta, double* out) const override {
    Variable v = Variable::Pose(core::SE3()); fromAmbient(x, v);
    v.plus(Eigen::Map<const core::Vec6>(delta)); toAmbient(v, out); return true;
  }
  bool PlusJacobian(const double*, double* jac) const override {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> J(jac); J.setZero(); J.topRows<6>().setIdentity(); return true;
  }
  bool Minus(const double* y, const double* x, double* out) const override {
    Variable vy = Variable::Pose(core::SE3()), vx = vy; fromAmbient(y, vy); fromAmbient(x, vx);
    Eigen::Map<core::Vec6> m(out); m = vy.minus(vx); return true;
  }
  bool MinusJacobian(const double*, double* jac) const override {
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> J(jac); J.setZero(); J.leftCols<6>().setIdentity(); return true;
  }
};

class GravityDirManifold final : public ceres::Manifold {
 public:
  int AmbientSize() const override { return 4; }
  int TangentSize() const override { return 2; }
  bool Plus(const double* x, const double* delta, double* out) const override {
    Variable v = Variable::GravityDir(core::SO3()); fromAmbient(x, v);
    v.plus(Eigen::Map<const Eigen::Vector2d>(delta)); toAmbient(v, out); return true;
  }
  bool PlusJacobian(const double*, double* jac) const override {
    Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::RowMajor>> J(jac); J.setZero(); J.topRows<2>().setIdentity(); return true;
  }
  bool Minus(const double* y, const double* x, double* out) const override {
    Variable vy = Variable::GravityDir(core::SO3()), vx = vy; fromAmbient(y, vy); fromAmbient(x, vx);
    Eigen::Map<Eigen::Vector2d> m(out); m = vy.minus(vx); return true;
  }
  bool MinusJacobian(const double*, double* jac) const override {
    Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> J(jac); J.setZero(); J.leftCols<2>().setIdentity(); return true;
  }
};

// Generic cost function forwarding to IFactor::evaluate.
class FactorCost final : public ceres::CostFunction {
 public:
  FactorCost(const IFactor& f, std::vector<Variable> templates) : f_(f), tmpl_(std::move(templates)) {
    set_num_residuals(f.residualDim());
    for (const auto& v : tmpl_) mutable_parameter_block_sizes()->push_back(ambientSize(v));
  }
  bool Evaluate(double const* const* params, double* residuals, double** jacobians) const override {
    Values vals;
    const auto& keys = f_.keys();
    for (std::size_t i = 0; i < keys.size(); ++i) { Variable v = tmpl_[i]; fromAmbient(params[i], v); vals.emplace(keys[i], std::move(v)); }
    Eigen::Map<Eigen::VectorXd> r(residuals, f_.residualDim());
    std::vector<Eigen::MatrixXd> J;
    if (!f_.evaluate(vals, r, jacobians ? &J : nullptr)) return false;
    if (jacobians) {
      for (std::size_t i = 0; i < keys.size(); ++i) {
        if (!jacobians[i]) continue;
        const int amb = ambientSize(tmpl_[i]), tan = tmpl_[i].dim();
        Eigen::Map<core::RowMatX> Ja(jacobians[i], f_.residualDim(), amb);
        Ja.setZero();
        Ja.leftCols(tan) = J[i];
      }
    }
    return true;
  }
 private:
  const IFactor& f_;
  std::vector<Variable> tmpl_;
};

class StopCallback final : public ceres::IterationCallback {
 public:
  explicit StopCallback(std::stop_token st) : st_(std::move(st)) {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary&) override {
    return st_.stop_requested() ? ceres::SOLVER_TERMINATE_SUCCESSFULLY : ceres::SOLVER_CONTINUE;
  }
  bool triggered() const { return st_.stop_requested(); }
 private:
  std::stop_token st_;
};

ceres::LinearSolverType toCeres(LinearSolverType t) {
  switch (t) {
    case LinearSolverType::DenseQr: return ceres::DENSE_QR;
    case LinearSolverType::DenseSchur: return ceres::DENSE_SCHUR;
    case LinearSolverType::SparseSchur: return ceres::SPARSE_SCHUR;
    case LinearSolverType::SparseNormalCholesky: return ceres::SPARSE_NORMAL_CHOLESKY;
  }
  return ceres::SPARSE_SCHUR;
}

}  // namespace

SolveReport CeresSolver::solve(Problem& problem, const SolveOptions& o, std::stop_token stop) {
  const auto t0 = std::chrono::steady_clock::now();
  SolveReport rep;
  rep.initial_chi2 = problem.totalChi2();

  ceres::Problem::Options popt;
  popt.cost_function_ownership = ceres::TAKE_OWNERSHIP;
  popt.loss_function_ownership = ceres::TAKE_OWNERSHIP;
  popt.manifold_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
  ceres::Problem cp(popt);
  Se3Manifold se3_manifold;
  GravityDirManifold gdir_manifold;

  std::unordered_map<core::VariableId, std::vector<double>> blocks;
  auto block = [&](core::VariableId id) -> double* {
    auto it = blocks.find(id);
    if (it != blocks.end()) return it->second.data();
    const Variable& v = problem.variable(id);
    auto& b = blocks[id];
    b.resize(static_cast<std::size_t>(ambientSize(v)));
    toAmbient(v, b.data());
    if (v.kind == VariableKind::Pose) cp.AddParameterBlock(b.data(), 7, &se3_manifold);
    else if (v.kind == VariableKind::GravityDir) cp.AddParameterBlock(b.data(), 4, &gdir_manifold);
    else cp.AddParameterBlock(b.data(), v.dim());
    if (problem.isFixed(id)) cp.SetParameterBlockConstant(b.data());
    return b.data();
  };

  auto ordering = std::make_shared<ceres::ParameterBlockOrdering>();
  for (const auto& f : problem.factors()) {
    std::vector<Variable> tmpl;
    std::vector<double*> params;
    for (auto k : f->keys()) { tmpl.push_back(problem.variable(k)); params.push_back(block(k)); }
    ceres::LossFunction* loss = nullptr;
    if (f->loss.type == RobustLoss::Type::Huber) loss = new ceres::HuberLoss(f->loss.delta);
    else if (f->loss.type == RobustLoss::Type::Cauchy) loss = new ceres::CauchyLoss(f->loss.delta);
    cp.AddResidualBlock(new FactorCost(*f, std::move(tmpl)), loss, params);
  }
  for (auto& [id, b] : blocks) {
    const auto kind = problem.variable(id).kind;
    ordering->AddElementToGroup(b.data(), (kind == VariableKind::Point || kind == VariableKind::InvDepth) ? 0 : 1);
  }

  ceres::Solver::Options so;
  so.linear_solver_type = toCeres(o.linear_solver);
  so.trust_region_strategy_type = o.algorithm == Algorithm::Dogleg ? ceres::DOGLEG : ceres::LEVENBERG_MARQUARDT;
  so.max_num_iterations = o.max_iterations;
  if (o.max_time_s > 0) so.max_solver_time_in_seconds = o.max_time_s;
  so.num_threads = o.num_threads;
  so.initial_trust_region_radius = 1.0 / std::max(o.initial_lambda, 1e-12);
  so.minimizer_progress_to_stdout = o.verbose;
  so.logging_type = o.verbose ? ceres::PER_MINIMIZER_ITERATION : ceres::SILENT;
  if (so.linear_solver_type == ceres::SPARSE_SCHUR || so.linear_solver_type == ceres::DENSE_SCHUR) {
    if (ordering->GroupSize(0) > 0) so.linear_solver_ordering = ordering;
    else so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  }
  StopCallback cb(std::move(stop));
  so.callbacks.push_back(&cb);

  ceres::Solver::Summary summary;
  ceres::Solve(so, &cp, &summary);

  for (auto& [id, b] : blocks) fromAmbient(b.data(), problem.variable(id));
  rep.converged = summary.termination_type == ceres::CONVERGENCE || summary.termination_type == ceres::USER_SUCCESS;
  rep.preempted = cb.triggered();
  rep.iterations = static_cast<int>(summary.iterations.size()) - 1;
  rep.final_chi2 = problem.totalChi2();
  rep.message = summary.BriefReport();
  rep.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return rep;
}

}  // namespace mvslam::backend::graph
