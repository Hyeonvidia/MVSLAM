#include "backend/marginalization/marginalizer.hpp"

#include <Eigen/Eigenvalues>
#include <set>

namespace mvslam::backend {

MarginalizationResult Marginalizer::marginalize(const graph::Problem& problem, const std::vector<core::VariableId>& marg_keys,
                                                const graph::Values* lin_point, double eig_eps) {
  MarginalizationResult res;
  const graph::Values& lin = lin_point ? *lin_point : problem.values();
  const std::set<core::VariableId> marg(marg_keys.begin(), marg_keys.end());
  std::set<core::VariableId> connected;
  auto touches = [&](const graph::IFactor& f) {
    for (auto k : f.keys()) if (marg.count(k)) return true;
    return false;
  };
  for (const auto& f : problem.factors()) {
    if (!touches(*f)) continue;
    for (auto k : f->keys()) if (!marg.count(k) && !problem.isFixed(k)) connected.insert(k);
  }
  std::vector<core::VariableId> order(marg_keys.begin(), marg_keys.end());
  order.insert(order.end(), connected.begin(), connected.end());
  const auto d = problem.linearizeDense(lin, order, touches);
  int m = 0;
  for (auto k : marg_keys) if (!problem.isFixed(k)) m += lin.at(k).dim();
  const int a = d.dim - m;
  res.marginalized_dim = m;
  res.remaining_keys.assign(connected.begin(), connected.end());
  if (a == 0) return res;

  const Eigen::MatrixXd Hmm = d.H.topLeftCorner(m, m);
  const Eigen::MatrixXd Hma = d.H.topRightCorner(m, a);
  const Eigen::MatrixXd Haa = d.H.bottomRightCorner(a, a);
  const Eigen::VectorXd gm = d.g.head(m), ga = d.g.tail(a);
  // Pseudo-inverse of Hmm through eigen-decomposition (rank-deficient safe, PSD projection).
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(0.5 * (Hmm + Hmm.transpose()));
  Eigen::VectorXd inv = es.eigenvalues();
  for (int i = 0; i < inv.size(); ++i) inv(i) = inv(i) > eig_eps ? 1.0 / inv(i) : 0.0;
  const Eigen::MatrixXd Hmm_inv = es.eigenvectors() * inv.asDiagonal() * es.eigenvectors().transpose();
  const Eigen::MatrixXd H_prior = Haa - Hma.transpose() * Hmm_inv * Hma;
  const Eigen::VectorXd g_prior = ga - Hma.transpose() * Hmm_inv * gm;

  graph::Values x0;
  for (auto k : res.remaining_keys) x0[k] = lin.at(k);
  res.prior = std::make_unique<graph::PriorFactor>(res.remaining_keys, std::move(x0), H_prior, g_prior, eig_eps);
  return res;
}

}  // namespace mvslam::backend
