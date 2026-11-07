#pragma once

#include <array>
#include <string>

#include "hybrid_astar/vehicle_parameter.h"
#include "optimizer/trajectory.h"

namespace optimizer {

struct FootprintValidity {
  double arc = 0.0;
  double forward_cond1 = 0.0;
  double forward_cond2 = 0.0;
  double reverse_cond1 = 0.0;
  double reverse_cond2 = 0.0;

  double max_violation() const;
};

FootprintInterval ComputeEmbodiedFootprint(const TrajectoryNode& node,
                                           double dt,
                                           const VehicleParameter& vehicle);

FootprintValidity EvaluateFootprintValidity(
    const FootprintInterval& footprint, const VehicleParameter& vehicle,
    double eta = 1.0);

bool CheckVehicleGeometryCondition(const VehicleParameter& vehicle,
                                   std::string* error);

bool WriteGeometryValidation(const VehicleParameter& vehicle,
                             const std::string& output_file,
                             std::string* error);

}  // namespace optimizer
