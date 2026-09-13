#include "backend/solvers/solver.hpp"

#include "mvslam/api/errors.hpp"
#ifdef MVSLAM_WITH_CERES
#include "backend/solvers/ceres/ceres_solver.hpp"
#endif

namespace mvslam::backend::graph {

std::unique_ptr<ISolver> makeSolver(SolverBackend backend) {
  switch (backend) {
    case SolverBackend::Ceres:
#ifdef MVSLAM_WITH_CERES
      return std::make_unique<CeresSolver>();
#else
      throw ConfigError("solver backend 'ceres' not compiled in (MVSLAM_WITH_CERES=OFF)");
#endif
    case SolverBackend::G2o: throw ConfigError("solver backend 'g2o' adapter not implemented yet (P4)");
    case SolverBackend::Gtsam: throw ConfigError("solver backend 'gtsam' adapter not implemented yet (P4)");
  }
  throw ConfigError("unknown solver backend");
}

}  // namespace mvslam::backend::graph
