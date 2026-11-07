#pragma once

#include <array>
#include <string>
#include <vector>

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

struct RejectedInterval {
  std::size_t interval_index = 0;
  std::size_t dense_start = 0;
  std::size_t dense_end = 0;
  double dt = 0.0;
  double curvature = 0.0;
  double signed_distance = 0.0;
  std::string violated_conditions;
  double max_violation = 0.0;
  std::array<double, 5> violations{};
};

struct InitialGuessDiagnostics {
  bool success = false;
  std::string failure_reason;
  std::size_t hybrid_point_count = 0;
  std::size_t deduplicated_point_count = 0;
  double hybrid_path_length = 0.0;
  std::size_t raw_direction_segments = 0;
  std::size_t effective_direction_segments = 0;
  std::size_t raw_gear_switches = 0;
  std::size_t effective_gear_switches = 0;
  std::vector<std::string> merged_short_segments;
  std::size_t dense_node_count = 0;
  std::size_t final_node_count = 0;
  double min_dt = 0.0;
  double max_dt = 0.0;
  double total_time = 0.0;
  double max_abs_velocity = 0.0;
  double max_abs_acceleration = 0.0;
  double max_abs_steering = 0.0;
  double max_abs_raw_steering_rate = 0.0;
  double max_abs_clipped_steering_rate = 0.0;
  double max_euler_residual = 0.0;
  double max_footprint_violation = 0.0;
  double endpoint_pose_error = 0.0;
  double max_velocity_bound_violation = 0.0;
  double max_acceleration_bound_violation = 0.0;
  double max_steering_bound_violation = 0.0;
  double max_steering_rate_bound_violation = 0.0;
  std::size_t nonfinite_value_count = 0;
  std::size_t direction_sign_mismatch_count = 0;
  std::size_t acceleration_clip_count = 0;
  std::size_t steering_rate_clip_count = 0;
  std::vector<RejectedInterval> rejected_intervals;
};

bool BuildInitialGuess(const planning::HybridAStartResult& hybrid_path,
                       const VehicleParameter& vehicle,
                       const InitialGuessConfig& config,
                       Trajectory* trajectory, InitialGuessDiagnostics* diagnostics,
                       std::string* error);

bool SaveInitialGuessDiagnostics(const std::string& output_file,
                                 const InitialGuessDiagnostics& diagnostics,
                                 std::string* error);

}  // namespace optimizer
