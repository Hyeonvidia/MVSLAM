#pragma once
// Ceres adapter: wraps IFactor as a dynamic ceres::CostFunction and Variable manifolds as
// ceres::Manifold (right-multiplicative SE3, 2-DoF gravity). Only this directory includes Ceres.
#include "backend/solvers/solver.hpp"

namespace mvslam::backend::graph {

class CeresSolver final : public ISolver {
 public:
  std::string name() const override { return "ceres"; }
  SolveReport solve(Problem& problem, const SolveOptions& options, std::stop_token stop) override;
};

}  // namespace mvslam::backend::graph
