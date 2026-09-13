#include <gtest/gtest.h>

#include "backend/marginalization/marginalizer.hpp"

using namespace mvslam::core;
using namespace mvslam::backend;
using namespace mvslam::backend::graph;

// Linear factor r = sqrt(W) (a - b - m) between two 3-D variables (a "relative measurement").
class LinearBetween final : public IFactor {
 public:
  LinearBetween(VariableId a, VariableId b, Vec3 m, double w) : keys_{a, b}, m_(m), sw_(std::sqrt(w)) {}
  const std::vector<VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 3; }
  std::string name() const override { return "LinearBetween"; }
  bool evaluate(const Values& v, Eigen::Ref<Eigen::VectorXd> r, std::vector<Eigen::MatrixXd>* J) const override {
    r = sw_ * (v.at(keys_[0]).vec - v.at(keys_[1]).vec - m_);
    if (J) { J->resize(2); (*J)[0] = sw_ * Eigen::Matrix3d::Identity(); (*J)[1] = -sw_ * Eigen::Matrix3d::Identity(); }
    return true;
  }
 private:
  std::vector<VariableId> keys_;
  Vec3 m_;
  double sw_;
};

class LinearPrior final : public IFactor {  // r = sqrt(W) (a - m)
 public:
  LinearPrior(VariableId a, Vec3 m, double w) : keys_{a}, m_(m), sw_(std::sqrt(w)) {}
  const std::vector<VariableId>& keys() const override { return keys_; }
  int residualDim() const override { return 3; }
  std::string name() const override { return "LinearPrior"; }
  bool evaluate(const Values& v, Eigen::Ref<Eigen::VectorXd> r, std::vector<Eigen::MatrixXd>* J) const override {
    r = sw_ * (v.at(keys_[0]).vec - m_);
    if (J) { J->resize(1); (*J)[0] = sw_ * Eigen::Matrix3d::Identity(); }
    return true;
  }
 private:
  std::vector<VariableId> keys_;
  Vec3 m_;
  double sw_;
};

// Chain a -- b -- c with a prior on a. Marginalizing a must leave the exact Gaussian on b (and c untouched).
TEST(Marginalizer, SchurComplementMatchesAnalyticChain) {
  const VariableId a{1}, b{2}, c{3};
  Problem p;
  p.addVariable(a, Variable::Point(Vec3(0.1, 0, 0)));
  p.addVariable(b, Variable::Point(Vec3(1.2, 0, 0)));
  p.addVariable(c, Variable::Point(Vec3(2.0, 0, 0)));
  p.addFactor(std::make_unique<LinearPrior>(a, Vec3(0, 0, 0), 4.0));
  p.addFactor(std::make_unique<LinearBetween>(b, a, Vec3(1, 0, 0), 2.0));
  p.addFactor(std::make_unique<LinearBetween>(c, b, Vec3(1, 0, 0), 1.0));
  auto res = Marginalizer::marginalize(p, {a});
  ASSERT_TRUE(res.prior);
  ASSERT_EQ(res.remaining_keys.size(), 1u);
  EXPECT_EQ(res.remaining_keys[0], b);
  // Analytic: information on b after marginalizing a = W_ab - W_ab (W_a + W_ab)^-1 W_ab = 2 - 4/6 = 4/3.
  EXPECT_NEAR(res.prior->H()(0, 0), 4.0 / 3.0, 1e-9);
  EXPECT_NEAR(res.prior->H()(1, 1), 4.0 / 3.0, 1e-9);
  // Gradient at the linearization point: g_b - H_ba H_aa^-1 g_a with r_a=4*0.1*... check via consistency:
  // the prior's minimizer must equal the marginal mean of b given the measurements (b* = 1).
  Problem q;
  q.addVariable(b, p.variable(b));
  q.addFactor(std::move(res.prior));
  // Solve the 1-factor quadratic analytically: d = -H^-1 g.
  const auto d = q.linearizeDense(q.values(), {b});
  Eigen::Vector3d step = -d.H.ldlt().solve(d.g);
  EXPECT_NEAR(q.variable(b).vec(0) + step(0), 1.0, 1e-9);
}
