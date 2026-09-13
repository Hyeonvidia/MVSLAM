#include "backend/graph/problem.hpp"

namespace mvslam::backend::graph {

double Problem::totalChi2() const {
  double s = 0;
  for (const auto& f : factors_) s += f->chi2(values_);
  return s;
}

Problem::Dense Problem::linearizeDense(const Values& lin, const std::vector<core::VariableId>& order,
                                       const std::function<bool(const IFactor&)>& include) const {
  Dense d;
  d.order = order;
  for (auto id : order) {
    if (fixed_.count(id)) continue;
    const int n = lin.at(id).dim();
    d.offset[id] = d.dim;
    d.dims[id] = n;
    d.dim += n;
  }
  d.H = Eigen::MatrixXd::Zero(d.dim, d.dim);
  d.g = Eigen::VectorXd::Zero(d.dim);
  for (const auto& f : factors_) {
    if (include && !include(*f)) continue;
    const auto& keys = f->keys();
    Eigen::VectorXd r(f->residualDim());
    std::vector<Eigen::MatrixXd> J;
    if (!f->evaluate(lin, r, &J)) continue;
    const double chi2 = r.squaredNorm();
    const double w = f->loss.weight(chi2);
    const double sw = std::sqrt(w);
    d.chi2 += w * chi2;
    for (std::size_t i = 0; i < keys.size(); ++i) {
      auto oi = d.offset.find(keys[i]);
      if (oi == d.offset.end()) continue;
      const Eigen::MatrixXd Ji = sw * J[i];
      d.g.segment(oi->second, Ji.cols()) += Ji.transpose() * (sw * r);
      for (std::size_t j = 0; j < keys.size(); ++j) {
        auto oj = d.offset.find(keys[j]);
        if (oj == d.offset.end()) continue;
        const Eigen::MatrixXd Jj = sw * J[j];
        d.H.block(oi->second, oj->second, Ji.cols(), Jj.cols()) += Ji.transpose() * Jj;
      }
    }
  }
  return d;
}

}  // namespace mvslam::backend::graph
