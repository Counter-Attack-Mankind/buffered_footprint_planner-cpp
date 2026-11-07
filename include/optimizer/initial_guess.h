#pragma once

#include <string>

#include "hybrid_astar/hybrid_a_star.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace optimizer {

struct InitialGuessConfig {
  double min_dt = 0.02;
  double max_dt = 0.25;
  double eta = 0.9;
  double forward_speed = 0.20;
  double reverse_speed = 0.16;
  double max_acceleration = 0.10;
  double max_steering_rate = 0.10;
};

bool BuildInitialGuess(const planning::HybridAStartResult& hybrid_path,
                       const VehicleParameter& vehicle,
                       const InitialGuessConfig& config,
                       Trajectory* trajectory, std::string* error);

}  // namespace optimizer
