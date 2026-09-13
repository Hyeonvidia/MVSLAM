#include "backend/imu/imu_initializer.hpp"

#include <algorithm>

#include "backend/imu/inertial_optimization.hpp"
#include "core/log.hpp"

namespace mvslam::backend::imu {

namespace {
// Keyframes in id order with the preintegration that links each one to its predecessor in the list.
std::vector<KeyframeInertialState> chainOf(const map::Map& m) {
  std::vector<KeyframeInertialState> chain;
  for (const auto& kf : m.keyframes()) {
    if (kf->bad()) continue;
    KeyframeInertialState s;
    s.id = kf->id(); s.timestamp = kf->timestamp(); s.T_wb = kf->T_wb(); s.v_w = kf->velocity(); s.bias = kf->bias();
    if (!chain.empty() && kf->prevKf() == chain.back().id) s.preint = kf->preintegration();
    chain.push_back(s);
  }
  return chain;
}
}  // namespace

ImuInitializer::ImuInitializer(const Config& cfg, const sensors::CameraRig& rig, std::shared_ptr<graph::ISolver> solver)
    : cfg_(cfg), rig_(rig), solver_(std::move(solver)), ba_(cfg, rig, solver_) {}

bool ImuInitializer::runStage(map::Map& map, double prior_gyro, double prior_acc, bool scale_free, int ba_iterations, ImuInitStep& step) {
  const auto chain = map.withReadLock([](const map::Map& m) { return chainOf(m); });
  if (chain.size() < 2) return false;
  const bool mono = cfg_.sensor == SensorMode::Mono;
  InertialOnlyOptions opt;
  opt.prior_gyro = prior_gyro;
  opt.prior_acc = prior_acc;
  opt.optimize_scale = mono && scale_free;
  opt.gravity = rig_.hasImu() ? rig_.imu().gravity : 9.81;
  const bool first = state_ == ImuInitState::Uninitialized;
  const core::SO3 R_wg0 = first ? initialGravityDirection(chain) : core::SO3();  // after alignment gravity is already -z
  core::Vec6 bias0 = chain.back().bias;
  auto work = chain;
  if (first) {
    const auto v = initialVelocities(chain);
    for (std::size_t i = 0; i < work.size(); ++i) work[i].v_w = v[i];
    bias0.setZero();
  }
  const auto res = inertialOnlyOptimization(work, R_wg0, 1.0, bias0, opt, *solver_);
  if (!res.ok) return false;
  if (opt.optimize_scale && res.scale < 0.1) { MVSLAM_WARN("imu init: scale {:.3f} too small, retrying later", res.scale); return false; }
  step.scale = res.scale;
  step.scale_variance = res.scale_variance;
  const double s = opt.optimize_scale ? res.scale : 1.0;
  const core::SO3 R_gw = res.R_wg.inverse();
  const bool apply = first || std::abs(s - 1.0) > 0.002 || R_gw.log().norm() > 1e-3;
  map.withWriteLock([&](map::Map& m) {
    for (std::size_t i = 0; i < chain.size(); ++i) {
      auto kf = m.keyframe(chain[i].id);
      if (!kf) continue;
      kf->setVelocity(res.velocities[i]);
      kf->setBias(res.bias);
    }
    if (apply) m.applyScaledRotation(s, R_gw);
    auto& st = m.imuState();
    st.initialized = true;
    st.scale *= s;
    return 0;
  });
  step.applied_scaled_rotation = apply;
  step.R_gw = apply ? R_gw : core::SO3();
  // Visual-inertial BA over the whole chain with the stage's bias priors (ORB-SLAM3 FullInertialBA, bInit).
  std::vector<core::KeyframeId> all;
  for (const auto& c : chain) all.push_back(c.id);
  InertialBaOptions bo;
  bo.iterations = ba_iterations;
  bo.bias_prior_gyro = prior_gyro;
  bo.bias_prior_acc = prior_acc;
  const auto rep = ba_.run(map, all, bo);
  MVSLAM_INFO("imu init stage: {} keyframes, scale {:.4f} (var {:.2e}), gravity correction {:.2f} deg, bias g [{:.4f} {:.4f} {:.4f}] a [{:.3f} {:.3f} {:.3f}], VI-BA cost {:.1f}->{:.1f}",
              chain.size(), s, res.scale_variance, R_gw.log().norm() * 180.0 / M_PI, res.bias(0), res.bias(1), res.bias(2), res.bias(3), res.bias(4),
              res.bias(5), rep.initial_cost, rep.final_cost);
  return true;
}

ImuInitStep ImuInitializer::step(map::Map& map, core::KeyframeId latest_kf) {
  ImuInitStep st;
  st.before = state_;
  const auto info = map.withReadLock([&](const map::Map& m) {
    struct { int n = 0; double span = 0; std::int64_t t_first = 0, t_latest = 0; bool initialized = false; std::int64_t t_init = 0; int refinements = 0; } r;
    const auto kfs = m.keyframes();
    r.n = static_cast<int>(kfs.size());
    if (!kfs.empty()) { r.t_first = kfs.front()->timestamp(); r.t_latest = kfs.back()->timestamp(); r.span = static_cast<double>(r.t_latest - r.t_first) * 1e-9; }
    if (auto kf = m.keyframe(latest_kf)) r.t_latest = kf->timestamp();
    r.initialized = m.imuState().initialized;
    r.t_init = m.imuState().t_init;
    r.refinements = m.imuState().scale_refinements;
    return r;
  });
  if (!info.initialized && state_ != ImuInitState::Uninitialized) { state_ = ImuInitState::Uninitialized; st.before = state_; }  // new map
  const bool mono = cfg_.sensor == SensorMode::Mono;
  const auto& b = cfg_.backend;
  const double since_init = info.initialized ? static_cast<double>(info.t_latest - info.t_init) * 1e-9 : 0.0;
  switch (state_) {
    case ImuInitState::Uninitialized: {
      const double min_span = mono ? b.imu_init_min_span_mono_s : b.imu_init_min_span_stereo_s;
      if (info.n < b.imu_init_min_keyframes || info.span < min_span) break;
      if (runStage(map, 1e2, mono ? 1e10 : 1e5, true, 20, st)) {
        map.withWriteLock([&](map::Map& m) { m.imuState().t_init = info.t_latest; return 0; });
        state_ = ImuInitState::Initialized;
        MVSLAM_INFO("imu initialized after {:.1f} s / {} keyframes (scale {:.3f})", info.span, info.n, st.scale);
      }
      break;
    }
    case ImuInitState::Initialized:
      if (since_init > b.imu_refine1_s && runStage(map, 1.0, 1e5, true, 10, st)) {
        map.withWriteLock([&](map::Map& m) { m.imuState().refined1 = true; return 0; });
        state_ = ImuInitState::Refine1;
      }
      break;
    case ImuInitState::Refine1:
      if (since_init > b.imu_refine2_s && runStage(map, 0.0, 0.0, true, 10, st)) {
        map.withWriteLock([&](map::Map& m) { m.imuState().refined2 = true; return 0; });
        state_ = mono ? ImuInitState::Refine2 : ImuInitState::Done;
      }
      break;
    case ImuInitState::Refine2: {  // mono scale refinement at 25 / 35 / 45 s (gravity + scale only)
      const double due = 25.0 + 10.0 * info.refinements;
      if (info.refinements >= 3) { state_ = ImuInitState::Done; break; }
      if (since_init < due) break;
      const auto chain = map.withReadLock([](const map::Map& m) { return chainOf(m); });
      InertialOnlyOptions opt;
      opt.prior_gyro = 0; opt.prior_acc = 0; opt.optimize_bias = false; opt.optimize_velocities = false; opt.optimize_scale = true; opt.optimize_gravity = true;
      opt.iterations = 10;
      opt.gravity = rig_.hasImu() ? rig_.imu().gravity : 9.81;
      const auto res = inertialOnlyOptimization(chain, core::SO3(), 1.0, chain.back().bias, opt, *solver_);
      st.scale = res.scale; st.scale_variance = res.scale_variance;
      if (res.ok && std::abs(res.scale - 1.0) > 0.002) {
        const core::SO3 R_gw = res.R_wg.inverse();
        map.withWriteLock([&](map::Map& m) { m.applyScaledRotation(res.scale, R_gw); m.imuState().scale *= res.scale; return 0; });
        st.applied_scaled_rotation = true; st.R_gw = R_gw;
        MVSLAM_INFO("imu scale refinement #{}: scale {:.4f} (var {:.2e})", info.refinements + 1, res.scale, res.scale_variance);
      }
      map.withWriteLock([&](map::Map& m) { ++m.imuState().scale_refinements; return 0; });
      break;
    }
    case ImuInitState::Done: break;
  }
  st.after = state_;
  return st;
}

}  // namespace mvslam::backend::imu
