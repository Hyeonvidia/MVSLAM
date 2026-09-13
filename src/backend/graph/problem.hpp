#pragma once
// Library-independent problem: variables, factors, fixed set, and dense linearization used by the
// Marginalizer and by tests. Solver adapters consume it through ISolver.
#include <functional>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "backend/graph/factor.hpp"

namespace mvslam::backend::graph {

class Problem {
 public:
  void addVariable(core::VariableId id, Variable v) { values_[id] = std::move(v); }
  bool hasVariable(core::VariableId id) const { return values_.count(id) != 0; }
  Variable& variable(core::VariableId id) { return values_.at(id); }
  const Variable& variable(core::VariableId id) const { return values_.at(id); }
  Values& values() { return values_; }
  const Values& values() const { return values_; }
  void setFixed(core::VariableId id, bool fixed = true) { if (fixed) fixed_.insert(id); else fixed_.erase(id); }
  bool isFixed(core::VariableId id) const { return fixed_.count(id) != 0; }
  const std::set<core::VariableId>& fixed() const { return fixed_; }
  void addFactor(std::unique_ptr<IFactor> f) { factors_.push_back(std::move(f)); }
  const std::vector<std::unique_ptr<IFactor>>& factors() const { return factors_; }
  std::vector<std::unique_ptr<IFactor>>& factors() { return factors_; }
  double totalChi2() const;
  double totalRobustCost() const;  // sum of rho(chi2); non-evaluable factors count as their delta^2 (finite)

  struct Dense {
    std::vector<core::VariableId> order;
    std::unordered_map<core::VariableId, int> offset;
    std::unordered_map<core::VariableId, int> dims;
    Eigen::MatrixXd H;   // sum J^T W J
    Eigen::VectorXd g;   // sum J^T W r
    double chi2 = 0;
    int dim = 0;
  };
  // Dense normal equations over `order` (keys not in `order` or fixed are treated as constants)
  // linearized at `lin_point`. `include` selects the factors (default: all).
  Dense linearizeDense(const Values& lin_point, const std::vector<core::VariableId>& order,
                       const std::function<bool(const IFactor&)>& include = nullptr) const;

 private:
  Values values_;
  std::set<core::VariableId> fixed_;
  std::vector<std::unique_ptr<IFactor>> factors_;
};

}  // namespace mvslam::backend::graph
