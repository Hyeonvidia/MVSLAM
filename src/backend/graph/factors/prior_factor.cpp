#include "backend/graph/factors/prior_factor.hpp"

#include <Eigen/Eigenvalues>

namespace mvslam::backend::graph {

PriorFactor::PriorFactor(std::vector<core::VariableId> keys, Values x0, const Eigen::MatrixXd& H,
                         const Eigen::VectorXd& g, double eig_eps)
    : keys_(std::move(keys)), x0_(std::move(x0)), H_(0.5 * (H + H.transpose())), g_(g) {
  int off = 0;
  for (auto k : keys_) { const int n = x0_.at(k).dim(); offsets_.push_back(off); dims_.push_back(n); off += n; }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(H_);
  Eigen::VectorXd lam = es.eigenvalues();
  Eigen::VectorXd sqrt_lam = Eigen::VectorXd::Zero(lam.size()), inv_sqrt_lam = Eigen::VectorXd::Zero(lam.size());
  for (int i = 0; i < lam.size(); ++i) {
    if (lam(i) > eig_eps) { sqrt_lam(i) = std::sqrt(lam(i)); inv_sqrt_lam(i) = 1.0 / sqrt_lam(i); ++rank_; }
  }
  const Eigen::MatrixXd Vt = es.eigenvectors().transpose();
  L_ = sqrt_lam.asDiagonal() * Vt;              // H = L^T L (zero rows for null directions)
  c_ = inv_sqrt_lam.asDiagonal() * Vt * g_;     // L^T c = g on the range of H
  H_ = L_.transpose() * L_;                      // PSD-projected H
}

bool PriorFactor::evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                           std::vector<Eigen::MatrixXd>* jacobians) const {
  Eigen::VectorXd d(L_.cols());
  for (std::size_t i = 0; i < keys_.size(); ++i)
    d.segment(offsets_[i], dims_[i]) = values.at(keys_[i]).minus(x0_.at(keys_[i]));
  residual = L_ * d + c_;
  if (jacobians) {
    jacobians->resize(keys_.size());
    for (std::size_t i = 0; i < keys_.size(); ++i) (*jacobians)[i] = L_.middleCols(offsets_[i], dims_[i]);
  }
  return true;
}

}  // namespace mvslam::backend::graph
