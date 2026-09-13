#pragma once
// A factor owns its residual and analytic Jacobians; solver adapters only forward calls to
// evaluate(). Residuals and Jacobians are *whitened* (sqrt information already applied).
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "backend/graph/variables.hpp"

namespace mvslam::backend::graph {

struct RobustLoss {
  enum class Type { None, Huber, Cauchy } type = Type::None;
  double delta = 1.0;  // in whitened residual units (sqrt(chi2 threshold))
  static RobustLoss None() { return {}; }
  static RobustLoss Huber(double d) { return {Type::Huber, d}; }
  static RobustLoss Cauchy(double d) { return {Type::Cauchy, d}; }
  // IRLS weight w(chi2) such that scaling r and J by sqrt(w) reproduces the robust normal equations.
  double weight(double chi2) const {
    switch (type) {
      case Type::None: return 1.0;
      case Type::Huber: return chi2 <= delta * delta ? 1.0 : delta / std::sqrt(chi2);
      case Type::Cauchy: return 1.0 / (1.0 + chi2 / (delta * delta));
    }
    return 1.0;
  }
};

class IFactor {
 public:
  virtual ~IFactor() = default;
  virtual const std::vector<core::VariableId>& keys() const = 0;
  virtual int residualDim() const = 0;
  virtual std::string name() const = 0;
  // Fills `residual` (residualDim) and, when `jacobians` is non-null, one residualDim x dim(key)
  // matrix per key (tangent space of the variable). Returns false if the point is not evaluable
  // (e.g. behind the camera) — the solver treats it as an invalid step.
  virtual bool evaluate(const Values& values, Eigen::Ref<Eigen::VectorXd> residual,
                        std::vector<Eigen::MatrixXd>* jacobians) const = 0;
  double chi2(const Values& values) const {
    Eigen::VectorXd r(residualDim());
    if (!evaluate(values, r, nullptr)) return std::numeric_limits<double>::infinity();
    return r.squaredNorm();
  }
  RobustLoss loss;
};

}  // namespace mvslam::backend::graph
