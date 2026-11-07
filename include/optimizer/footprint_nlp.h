#pragma once

#include <array>
#include <string>
#include <vector>

#include "common/math/polygon2d.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace optimizer {

struct FootprintNlpConfig {
  double min_dt = 0.02;
  double max_dt = 0.25;
  double total_time_limit = 35.0;
  double dmin = 0.005;
  double tolerance = 1e-5;
  int max_iterations = 500;
  double max_cpu_time = 60.0;
  std::string linear_solver = "mumps";
};

OptimizationResult SolveFootprintNlp(
    const Trajectory& initial, const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds,
    const FootprintNlpConfig& config);

bool SaveOptimizationResults(const std::string& result_directory,
                             const Trajectory& initial,
                             const OptimizationResult& result,
                             std::string* error);

bool WriteCollisionValidation(
    const std::string& output_file, const Trajectory& trajectory,
    const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds, std::string* error);

}  // namespace optimizer
