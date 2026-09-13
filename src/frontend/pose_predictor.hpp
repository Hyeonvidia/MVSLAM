#pragma once
// Constant-velocity motion model in se(3): T_pred = T_last * exp(v * dt) with v = log(T_prev^-1 T_last)/dt_prev.
#include "frontend/interfaces.hpp"

namespace mvslam::frontend {

class ConstantVelocityPredictor final : public IPosePredictor {
 public:
  core::SE3 predict(const Frame& last, Timestamp t_next) override {
    if (!has_velocity_) return last.T_wb;
    const double dt = static_cast<double>(t_next - last.timestamp) * 1e-9;
    return last.T_wb * core::SE3::exp(twist_ * dt);
  }
  void update(const Frame& last, const Frame& cur) override {
    const double dt = static_cast<double>(cur.timestamp - last.timestamp) * 1e-9;
    if (dt <= 0) return;
    twist_ = (last.T_wb.inverse() * cur.T_wb).log() / dt;
    has_velocity_ = true;
  }
  void reset() override { has_velocity_ = false; twist_.setZero(); }
 private:
  bool has_velocity_ = false;
  core::Vec6 twist_ = core::Vec6::Zero();
};

}  // namespace mvslam::frontend
