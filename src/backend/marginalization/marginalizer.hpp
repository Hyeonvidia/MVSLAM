#pragma once
// Library-independent Schur-complement marginalization producing a PriorFactor (FEJ at the
// linearization point). Used for the frame-to-frame pose-inertial prior and (optionally) sliding
// windows. docs/design/03-backend.md §1.4.
#include <memory>
#include <vector>

#include "backend/graph/factors/prior_factor.hpp"
#include "backend/graph/problem.hpp"

namespace mvslam::backend {

struct MarginalizationResult {
  std::unique_ptr<graph::PriorFactor> prior;   // over the connected (remaining) keys; null if none
  std::vector<core::VariableId> remaining_keys;
  int marginalized_dim = 0;
};

class Marginalizer {
 public:
  // Marginalizes `marg_keys` using only the factors that touch them (Markov blanket).
  // `lin_point` defaults to problem.values(); pass FEJ values to keep consistency.
  static MarginalizationResult marginalize(const graph::Problem& problem, const std::vector<core::VariableId>& marg_keys,
                                           const graph::Values* lin_point = nullptr, double eig_eps = 1e-8);
};

}  // namespace mvslam::backend
