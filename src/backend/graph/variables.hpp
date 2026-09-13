#pragma once
// Optimization variables with their manifolds (docs/design/03-backend.md §1.1).
// Pose increments are right-multiplied (body frame), scale is log-parameterized, gravity is 2-DoF.
#include <unordered_map>
#include <vector>

#include "core/ids.hpp"
#include "core/math/se3.hpp"

namespace mvslam::backend::graph {

enum class VariableKind { Pose, Velocity, Bias, Point, InvDepth, LogScale, GravityDir };

struct Variable {
  VariableKind kind = VariableKind::Point;
  core::SE3 pose;           // Pose
  core::SO3 rot;            // GravityDir: R_wg
  Eigen::VectorXd vec;      // Velocity(3), Bias(6), Point(3), InvDepth(1), LogScale(1)

  static Variable Pose(const core::SE3& T) { Variable v; v.kind = VariableKind::Pose; v.pose = T; return v; }
  static Variable Velocity(const core::Vec3& x) { Variable v; v.kind = VariableKind::Velocity; v.vec = x; return v; }
  static Variable Bias(const core::Vec6& x) { Variable v; v.kind = VariableKind::Bias; v.vec = x; return v; }
  static Variable Point(const core::Vec3& x) { Variable v; v.kind = VariableKind::Point; v.vec = x; return v; }
  static Variable InvDepth(double rho) { Variable v; v.kind = VariableKind::InvDepth; v.vec = Eigen::VectorXd::Constant(1, rho); return v; }
  static Variable LogScale(double sigma) { Variable v; v.kind = VariableKind::LogScale; v.vec = Eigen::VectorXd::Constant(1, sigma); return v; }
  static Variable GravityDir(const core::SO3& R) { Variable v; v.kind = VariableKind::GravityDir; v.rot = R; return v; }

  int dim() const {  // tangent dimension
    switch (kind) {
      case VariableKind::Pose: return 6;
      case VariableKind::Velocity: case VariableKind::Point: return 3;
      case VariableKind::Bias: return 6;
      case VariableKind::InvDepth: case VariableKind::LogScale: return 1;
      case VariableKind::GravityDir: return 2;
    }
    return 0;
  }
  void plus(const Eigen::Ref<const Eigen::VectorXd>& delta) {
    switch (kind) {
      case VariableKind::Pose: pose = pose * core::SE3::exp(delta.head<6>()); break;
      case VariableKind::GravityDir: rot = rot * core::SO3::exp(core::Vec3(delta(0), delta(1), 0.0)); break;
      default: vec += delta; break;
    }
  }
  // this ⊟ other, expressed in the tangent space at `other`.
  Eigen::VectorXd minus(const Variable& other) const {
    switch (kind) {
      case VariableKind::Pose: return (other.pose.inverse() * pose).log();
      case VariableKind::GravityDir: return (other.rot.inverse() * rot).log().head<2>();
      default: return vec - other.vec;
    }
  }
  double scale() const { return std::exp(vec(0)); }  // LogScale helper
};

using Values = std::unordered_map<core::VariableId, Variable>;

}  // namespace mvslam::backend::graph
