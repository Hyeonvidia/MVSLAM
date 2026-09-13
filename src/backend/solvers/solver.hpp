#pragma once
// Solver adapter interface. Every adapter (Ceres / g2o / GTSAM) consumes the same Problem and
// writes the optimized values back into it. docs/design/03-backend.md §1.3.
#include <functional>
#include <memory>
#include <stop_token>
#include <string>
#include <vector>

#include "backend/graph/problem.hpp"
#include "mvslam/api/config.hpp"

namespace mvslam::backend::graph {

enum class LinearSolverType { DenseQr, DenseSchur, SparseSchur, SparseNormalCholesky };
enum class Algorithm { LevenbergMarquardt, GaussNewton, Dogleg };

struct SolveOptions {
  int max_iterations = 10;
  double max_time_s = 0;              // 0 = unbounded (Offline mode)
  int num_threads = 1;
  LinearSolverType linear_solver = LinearSolverType::SparseSchur;
  Algorithm algorithm = Algorithm::LevenbergMarquardt;
  double initial_lambda = 1e-4;
  bool verbose = false;
  std::function<bool()> stop_predicate;  // optional: e.g. "a newer keyframe is waiting" (checked per iteration)
};

struct SolveReport {
  bool converged = false;
  bool preempted = false;
  int iterations = 0;
  double initial_chi2 = 0, final_chi2 = 0, time_s = 0;
  std::string message;
};

class ISolver {
 public:
  virtual ~ISolver() = default;
  virtual std::string name() const = 0;
  virtual SolveReport solve(Problem& problem, const SolveOptions& options, std::stop_token stop = {}) = 0;
};

// Factory: throws ConfigError if the requested backend was not compiled in.
std::unique_ptr<ISolver> makeSolver(SolverBackend backend);

}  // namespace mvslam::backend::graph
