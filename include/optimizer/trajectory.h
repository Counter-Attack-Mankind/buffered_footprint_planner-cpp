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
  bool initial_guess_succeeded = false;
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
  std::size_t dynamics_check_count = 0;
  std::size_t variable_bound_check_count = 0;
  std::size_t footprint_validity_check_count = 0;
  std::size_t obca_check_count = 0;
  std::size_t map_boundary_check_count = 0;
  std::string max_dynamics_location;
  std::string max_variable_bound_location;
  std::string max_footprint_validity_location;
  std::string max_obca_location;
  std::string max_map_boundary_location;
  int configured_max_iterations = 0;
  double configured_max_cpu_time = 0.0;
  double configured_tolerance = 0.0;
  std::string configured_linear_solver;
  std::string configured_hessian_approximation;
  std::string configured_mu_strategy;
  std::size_t model_variable_count = 0;
  std::size_t model_constraint_count = 0;
  std::size_t model_obstacle_count = 0;
  std::size_t model_obca_variable_count = 0;
  double model_build_time = 0.0;
};

}  // namespace optimizer
