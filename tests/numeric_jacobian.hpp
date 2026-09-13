#pragma once
// Central-difference Jacobian of an IFactor w.r.t. one key, in the variable's tangent space.
#include <Eigen/Core>

#include "backend/graph/factor.hpp"

namespace mvslam::testing {

inline Eigen::MatrixXd numericJacobian(const backend::graph::IFactor& f, const backend::graph::Values& values,
                                       core::VariableId key, double eps = 1e-6) {
  const int n = values.at(key).dim(), m = f.residualDim();
  Eigen::MatrixXd J(m, n);
  for (int i = 0; i < n; ++i) {
    Eigen::VectorXd d = Eigen::VectorXd::Zero(n);
    d(i) = eps;
    auto vp = values, vm = values;
    vp.at(key).plus(d);
    vm.at(key).plus(-d);
    Eigen::VectorXd rp(m), rm(m);
    f.evaluate(vp, rp, nullptr);
    f.evaluate(vm, rm, nullptr);
    J.col(i) = (rp - rm) / (2 * eps);
  }
  return J;
}

}  // namespace mvslam::testing
