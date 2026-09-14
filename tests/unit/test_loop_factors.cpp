#include <gtest/gtest.h>

#include <random>

#include "backend/graph/factors/relative_pose_factor.hpp"
#include "backend/loop/alignment.hpp"
#include "backend/loop/pose_graph_optimizer.hpp"
#include "backend/solvers/solver.hpp"
#include "map/map_store.hpp"
#include "numeric_jacobian.hpp"

using namespace mvslam::core;
using namespace mvslam::backend;
using mvslam::testing::numericJacobian;

namespace {
SE3 randomPose(std::mt19937& rng, double t = 1.0, double r = 0.5) {
  std::uniform_real_distribution<double> u(-1, 1);
  return SE3::exp((Vec6() << t * u(rng), t * u(rng), t * u(rng), r * u(rng), r * u(rng), r * u(rng)).finished());
}
void expectJacobians(const graph::IFactor& f, const graph::Values& v, double tol) {
  Eigen::VectorXd r(f.residualDim());
  std::vector<Eigen::MatrixXd> J;
  ASSERT_TRUE(f.evaluate(v, r, &J));
  for (std::size_t k = 0; k < f.keys().size(); ++k) {
    const Eigen::MatrixXd Jn = numericJacobian(f, v, f.keys()[k], 1e-6);
    EXPECT_LT((J[k] - Jn).norm() / std::max(1.0, Jn.norm()), tol) << f.name() << " key " << k << "\n" << J[k] << "\n---\n" << Jn;
  }
}
}  // namespace

TEST(RelativePoseSE3Factor, JacobiansMatchNumeric) {
  std::mt19937 rng(5);
  graph::Values v;
  const VariableId a{1}, b{2};
  v[a] = graph::Variable::Pose(randomPose(rng));
  v[b] = graph::Variable::Pose(randomPose(rng));
  const SE3 meas = v[a].pose.inverse() * v[b].pose * randomPose(rng, 0.1, 0.1);
  graph::RelativePoseSE3Factor f(a, b, meas, 2.0);
  expectJacobians(f, v, 1e-6);
  Eigen::VectorXd r(6);
  graph::RelativePoseSE3Factor exact(a, b, v[a].pose.inverse() * v[b].pose);
  exact.evaluate(v, r, nullptr);
  EXPECT_LT(r.norm(), 1e-12);
}

TEST(LoopReprojectionFactor, JacobiansMatchNumericBothDirections) {
  mvslam::CameraConfig cc; cc.width = 752; cc.height = 480; cc.intrinsics = {458.654, 457.296, 367.215, 248.375};
  auto cam = mvslam::sensors::makeCameraModel(cc);
  std::mt19937 rng(8);
  const SE3 T_cw = randomPose(rng, 0.3, 0.2);
  const SE3 T = randomPose(rng, 0.5, 0.3);
  graph::Values v;
  const VariableId key{1};
  v[key] = graph::Variable::Pose(T * randomPose(rng, 0.05, 0.05));
  for (bool inverse : {false, true}) {
    const Vec3 X(0.4, -0.3, 4.0);
    const Vec3 Y = inverse ? T.inverse() * X : T * X;
    const Vec2 uv = cam->project(T_cw * Y);
    graph::LoopReprojectionFactor f(key, *cam, T_cw, X, uv, inverse, 1.5);
    expectJacobians(f, v, 1e-6);
  }
}

TEST(Alignment, RansacRecoversSe3AndSim3WithOutliers) {
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1, 1);
  const SE3 T_gt = randomPose(rng, 2.0, 0.8);
  for (bool with_scale : {false, true}) {
    const double s_gt = with_scale ? 1.7 : 1.0;
    std::vector<Vec3> src, dst;
    for (int i = 0; i < 100; ++i) {
      const Vec3 p(3 * u(rng), 3 * u(rng), 3 * u(rng));
      src.push_back(p);
      Vec3 q = s_gt * (T_gt.so3() * p) + T_gt.translation() + Vec3(0.005 * u(rng), 0.005 * u(rng), 0.005 * u(rng));
      if (i % 4 == 0) q += Vec3(2 * u(rng), 2 * u(rng), 2 * u(rng)) + Vec3(1, 1, 1);  // 25% outliers
      dst.push_back(q);
    }
    loop::AlignmentOptions opt;
    opt.with_scale = with_scale;
    opt.inlier_dist_m = 0.05;
    const auto a = loop::alignPointsRansac(src, dst, opt);
    ASSERT_TRUE(a.ok);
    EXPECT_GE(a.inliers.size(), 70u);
    EXPECT_LE(a.inliers.size(), 80u);
    EXPECT_NEAR(a.scale, s_gt, 0.01);
    EXPECT_LT((a.T_dst_src.so3().inverse() * T_gt.so3()).log().norm(), 0.01);
    EXPECT_LT((a.T_dst_src.translation() - T_gt.translation()).norm(), 0.03);
  }
}

// A drifting chain of 40 keyframes that returns to its start: the loop edge pulls the last keyframe back
// onto the first and the pose graph distributes the drift over the chain.
TEST(PoseGraphOptimizer, DistributesLoopDrift) {
  mvslam::map::MapStore store;
  auto map = store.active();
  const int n = 40;
  std::vector<KeyframeId> ids;
  std::vector<SE3> gt;
  const double step_ang = 2 * M_PI / n;
  for (int i = 0; i < n; ++i) gt.emplace_back(SO3::exp(Vec3(0, 0, step_ang * i)), Vec3(3 * std::cos(step_ang * i), 3 * std::sin(step_ang * i), 0));
  // Odometry with a constant bias: each step rotates 1.5% short and translates 2% short -> accumulated drift.
  std::vector<SE3> drifted{gt[0]};
  for (int i = 1; i < n; ++i) {
    const SE3 rel = gt[static_cast<std::size_t>(i - 1)].inverse() * gt[static_cast<std::size_t>(i)];
    const SE3 rel_biased = SE3::exp(rel.log() * 0.985);
    drifted.push_back(drifted.back() * rel_biased);
  }
  map->withWriteLock([&](mvslam::map::Map& m) {
    for (int i = 0; i < n; ++i) {
      auto id = store.keyframeIds().next();
      auto kf = std::make_shared<mvslam::map::KeyFrame>(id, i, drifted[static_cast<std::size_t>(i)]);
      if (i > 0) { kf->setParent(ids.back()); kf->setPrevKf(ids.back()); }
      m.addKeyFrame(kf);
      ids.push_back(id);
    }
    // A few map points anchored at keyframe 5 to check point propagation.
    for (int p = 0; p < 3; ++p) m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(MapPointId{static_cast<std::uint64_t>(100 + p)}, drifted[5] * Vec3(p, 1, 2), ids[5]));
    return 0;
  });
  const SE3 drift_end = gt.back().inverse() * drifted.back();
  EXPECT_GT(drift_end.translation().norm(), 0.2);  // meaningful accumulated drift
  // Loop closure detected between the last keyframe and the first: the last keyframe's corrected pose is
  // the ground truth (as if aligned to the first keyframe's neighbourhood).
  std::unordered_map<KeyframeId, SE3> reference;
  for (int i = 0; i < n; ++i) reference[ids[static_cast<std::size_t>(i)]] = drifted[static_cast<std::size_t>(i)];
  map->withWriteLock([&](mvslam::map::Map& m) { m.keyframe(ids.back())->setT_wb(gt.back()); m.keyframe(ids.back())->loopEdges().insert(ids.front()); m.keyframe(ids.front())->loopEdges().insert(ids.back()); return 0; });
  auto solver = std::shared_ptr<graph::ISolver>(graph::makeSolver(mvslam::SolverBackend::Ceres));
  loop::PoseGraphOptimizer pgo(solver);
  loop::PgoOptions opt;
  const auto rep = pgo.optimize(*map, reference, {ids.front()}, {}, opt);
  ASSERT_TRUE(rep.applied);
  EXPECT_EQ(rep.nodes, n);
  EXPECT_EQ(rep.edges, n);  // chain + 1 loop edge
  EXPECT_LT(rep.final_chi2, rep.initial_chi2);
  map->withReadLock([&](const mvslam::map::Map& m) {
    double max_err = 0;
    for (int i = 0; i < n; ++i) max_err = std::max(max_err, (m.keyframe(ids[static_cast<std::size_t>(i)])->T_wb().translation() - gt[static_cast<std::size_t>(i)].translation()).norm());
    // The loop error is spread over the cycle (equal edge weights): the closing keyframe lands near its
    // corrected pose and no keyframe keeps more than half of the accumulated drift.
    EXPECT_LT((m.keyframe(ids.back())->T_wb().translation() - gt.back().translation()).norm(), 0.03);
    EXPECT_LT(max_err, 0.5 * drift_end.translation().norm()) << "drift " << drift_end.translation().norm() << " residual " << max_err;
    // Points followed their anchor (keyframe 5): still at the same position relative to it.
    const SE3 T5 = m.keyframe(ids[5])->T_wb();
    EXPECT_LT((T5.inverse() * m.mapPoint(MapPointId{101})->position() - Vec3(1, 1, 2)).norm(), 1e-9);
    return 0;
  });
}

TEST(Map, MergeMapPointsAndAbsorb) {
  mvslam::map::MapStore store;
  auto a = store.active();
  KeyframeId k1 = store.keyframeIds().next(), k2 = store.keyframeIds().next();
  a->withWriteLock([&](mvslam::map::Map& m) {
    for (auto k : {k1, k2}) { auto kf = std::make_shared<mvslam::map::KeyFrame>(k, 0, SE3()); kf->keypoints().resize(2); kf->mapPointSlots().assign(2, MapPointId{}); m.addKeyFrame(kf); }
    m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(MapPointId{10}, Vec3(0, 0, 1), k1));
    m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(MapPointId{11}, Vec3(0, 0, 1.01), k2));
    m.addObservation(MapPointId{10}, k1, {0, 0});
    m.addObservation(MapPointId{11}, k2, {1, 0});
    m.addObservation(MapPointId{11}, k1, {1, 0});
    m.mergeMapPoints(MapPointId{10}, MapPointId{11});
    EXPECT_EQ(m.mapPoint(MapPointId{11}), nullptr);
    EXPECT_EQ(m.mapPoint(MapPointId{10})->numObservations(), 2);   // k1 (kept its own slot 0) + k2
    EXPECT_EQ(m.keyframe(k2)->mapPointSlots()[1].value, 10u);
    EXPECT_FALSE(m.keyframe(k1)->mapPointSlots()[1].valid());      // duplicate observation dropped
    EXPECT_EQ(m.resolveTrack(TrackId{11}).value, 10u);
    return 0;
  });
  auto b = store.startNewMap();
  KeyframeId k3 = store.keyframeIds().next();
  b->withWriteLock([&](mvslam::map::Map& m) {
    auto kf = std::make_shared<mvslam::map::KeyFrame>(k3, 5, SE3(SO3(), Vec3(1, 0, 0)));
    kf->keypoints().resize(1); kf->mapPointSlots().assign(1, MapPointId{});
    m.addKeyFrame(kf);
    m.addMapPoint(std::make_shared<mvslam::map::MapPoint>(MapPointId{20}, Vec3(1, 0, 1), k3));
    m.addObservation(MapPointId{20}, k3, {0, 0});
    m.applyTransform(SE3(SO3(), Vec3(0, 0, 5)));
    EXPECT_NEAR(m.keyframe(k3)->T_wb().translation().z(), 5.0, 1e-12);
    EXPECT_NEAR(m.mapPoint(MapPointId{20})->position().z(), 6.0, 1e-12);
    // Similarity (monocular merge): X <- s R X + t, keyframe translations likewise, velocities scaled, culled
    // records follow (absolute pose transformed, relative translation scaled).
    m.keyframe(k3)->setVelocity(Vec3(1, 0, 0));
    m.rememberCulled(KeyframeId{99}, {k3, SE3(SO3(), Vec3(0.5, 0, 0)), SE3(SO3(), Vec3(1.5, 0, 5))});
    m.applyTransform(Sim3(Sophus::RxSO3d(2.0, SO3::exp(Vec3(0, 0, M_PI / 2))), Vec3(1, 0, 0)));
    EXPECT_LT((m.keyframe(k3)->T_wb().translation() - Vec3(1, 2, 10)).norm(), 1e-9);   // 2 * R * (1,0,5) + (1,0,0)
    EXPECT_LT((m.keyframe(k3)->velocity() - Vec3(0, 2, 0)).norm(), 1e-9);
    EXPECT_LT((m.mapPoint(MapPointId{20})->position() - Vec3(1, 2, 12)).norm(), 1e-9);
    const auto rec = m.culledRecords().at(KeyframeId{99});
    EXPECT_NEAR(rec.T_rc.translation().x(), 1.0, 1e-12);
    EXPECT_LT((rec.T_wc_at_cull.translation() - Vec3(1, 3, 10)).norm(), 1e-9);
    EXPECT_LT((m.resolvePose(KeyframeId{99})->translation() - (m.keyframe(k3)->T_wb() * SE3(SO3(), Vec3(1, 0, 0))).translation()).norm(), 1e-9);
    return 0;
  });
  a->withWriteLock([&](mvslam::map::Map& m) { b->withWriteLock([&](mvslam::map::Map& other) { m.absorb(other); return 0; }); return 0; });
  a->withReadLock([&](const mvslam::map::Map& m) { EXPECT_EQ(m.numKeyframes(), 3u); EXPECT_EQ(m.numMapPoints(), 2u); EXPECT_EQ(m.origins().size(), 2u); return 0; });
  b->withReadLock([&](const mvslam::map::Map& m) { EXPECT_EQ(m.numKeyframes(), 0u); return 0; });
}
