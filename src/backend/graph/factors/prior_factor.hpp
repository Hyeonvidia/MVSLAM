#pragma once
// Gaussian prior produced by marginalization: cost = g^T d + 1/2 d^T H d with d = x ⊟ x0 (FEJ).
// Implemented as r = L d + c with H = L^T L (eigen-decomposition, rank-deficient safe).
#include "backend/graph/factor.hpp"

namespace mvslam::backend::graph {

class PriorFactor final : public IFactor {
 public:
  PriorFactor(std::vector<core::VariableId> keys, Values x0, const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
              double eig_eps = 1e-8);
  const std::vector<core::VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return static_cast<int>(L_.rows()); }
  std::string name() const override { return "Prior"; }
  bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                std::vector<Eigen::MatrixXd>* jacobians) const override;
  const Values& linearizationPoint() const { return x0_; }
  const Eigen::MatrixXd& H() const { return H_; }
  const Eigen::VectorXd& g() const { return g_; }
  int rank() const { return rank_; }

 private:
  std::vector<core::VariableId> keys_;
  Values x0_;
  Eigen::MatrixXd H_, L_;
  Eigen::VectorXd g_, c_;
  std::vector<int> offsets_, dims_;
  int rank_ = 0;
};

}  // namespace mvslam::backend::graph
