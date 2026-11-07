#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/math/vec2d.h"

namespace optimizer {

struct TrajectoryNode {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double v = 0.0;
  double a = 0.0;
  double phy = 0.0;
  double w = 0.0;
};

struct FootprintInterval {
  double dt = 0.0;
  double s = 0.0;
  bool gear = true;
  double k = 0.0;
  double splus = 0.0;
  double sminus = 0.0;
  double up = 0.0;
  double down = 0.0;
  double left = 0.0;
  double right = 0.0;
  std::array<common::math::Vec2d, 4> corners;
};

struct Trajectory {
  std::vector<TrajectoryNode> nodes;
  std::vector<FootprintInterval> intervals;
};

struct OptimizationResult {
  Trajectory trajectory;
  bool solver_succeeded = false;
  bool success = false;
  std::string status;
  int iterations = 0;
  double solve_time = 0.0;
  double objective = 0.0;
  double max_dynamics_residual = 0.0;
  double max_variable_bound_violation = 0.0;
  double max_footprint_validity_violation = 0.0;
  double max_obca_violation = 0.0;
  double max_map_boundary_violation = 0.0;
};

}  // namespace optimizer
