#include "optimizer/initial_guess.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "optimizer/embodied_footprint.h"

namespace optimizer {
namespace {

struct PathSample {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  bool gear = true;
};

PathSample Interpolate(const std::vector<PathSample>& path,
                       const std::vector<double>& distance,
                       std::size_t first, std::size_t last, double query) {
  if (query <= 0.0) return path[first];
  const double end_distance = distance[last] - distance[first];
  if (query >= end_distance) return path[last];
  std::size_t upper = first + 1;
  while (upper <= last && distance[upper] - distance[first] < query) ++upper;
  const std::size_t lower = upper - 1;
  const double segment = distance[upper] - distance[lower];
  const double ratio = segment > 0.0
                           ? (query - (distance[lower] - distance[first])) /
                                 segment
                           : 0.0;
  PathSample result;
  result.x = path[lower].x + ratio * (path[upper].x - path[lower].x);
  result.y = path[lower].y + ratio * (path[upper].y - path[lower].y);
  result.theta = path[lower].theta +
                 ratio * (path[upper].theta - path[lower].theta);
  result.gear = path[first].gear;
  return result;
}

bool BuildWithStep(const std::vector<PathSample>& path,
                   const std::vector<double>& path_distance,
                   const VehicleParameter& vehicle,
                   const InitialGuessConfig& config, double dt,
                   Trajectory* trajectory, std::string* error) {
  trajectory->nodes.clear();
  trajectory->intervals.clear();
  std::size_t interval_start = 0;
  while (interval_start + 1 < path.size()) {
    const bool forward = path[interval_start].gear;
    std::size_t interval_end = interval_start + 1;
    while (interval_end + 1 < path.size() &&
           path[interval_end].gear == forward) {
      ++interval_end;
    }
    const double length = path_distance[interval_end] -
                          path_distance[interval_start];
    if (length <= 1e-10) {
      interval_start = interval_end;
      continue;
    }
    const double speed_limit = std::min(
        vehicle.v_max, forward ? config.forward_speed : config.reverse_speed);
    std::size_t count = 3;
    std::vector<double> velocity;
    double maximum_distance = 0.0;
    for (; count < 2000; ++count) {
      velocity.assign(count + 1, 0.0);
      maximum_distance = 0.0;
      for (std::size_t i = 0; i <= count; ++i) {
        const double accelerating =
            i > 0 ? (i - 1) * config.max_acceleration * dt : 0.0;
        const double decelerating =
            (count - i) * config.max_acceleration * dt;
        velocity[i] = std::min({speed_limit, accelerating, decelerating});
        if (i < count) maximum_distance += velocity[i] * dt;
      }
      if (maximum_distance + 1e-12 >= length) break;
    }
    if (count >= 2000 || maximum_distance <= 0.0) {
      if (error) *error = "Cannot allocate a bounded acceleration profile";
      return false;
    }
    const double scale = length / maximum_distance;
    for (double& value : velocity) value *= scale;

    std::vector<double> local_distance(count + 1, 0.0);
    for (std::size_t i = 0; i < count; ++i) {
      local_distance[i + 1] = local_distance[i] + velocity[i] * dt;
    }
    local_distance.back() = length;

    const std::size_t node_offset = trajectory->nodes.size();
    for (std::size_t i = 0; i <= count; ++i) {
      if (interval_start > 0 && i == 0) continue;
      const auto sample = Interpolate(path, path_distance, interval_start,
                                      interval_end, local_distance[i]);
      TrajectoryNode node;
      node.x = sample.x;
      node.y = sample.y;
      node.theta = sample.theta;
      node.v = (forward ? 1.0 : -1.0) * velocity[i];
      trajectory->nodes.push_back(node);
    }
    const std::size_t first_node = interval_start == 0
                                       ? node_offset
                                       : node_offset - 1;
    for (std::size_t i = 0; i < count; ++i) {
      const std::size_t node_index = first_node + i;
      auto& node = trajectory->nodes[node_index];
      const auto& next = trajectory->nodes[node_index + 1];
      const double signed_distance = node.v * dt;
      node.a = (next.v - node.v) / dt;
      node.phy = std::abs(signed_distance) > 1e-12
                     ? std::atan((next.theta - node.theta) *
                                 vehicle.wheel_base / signed_distance)
                     : 0.0;
      node.phy = std::clamp(node.phy, -vehicle.delta_max,
                            vehicle.delta_max);
      FootprintInterval footprint = ComputeEmbodiedFootprint(node, dt, vehicle);
      footprint.gear = forward;
      if (EvaluateFootprintValidity(footprint, vehicle, config.eta)
              .max_violation() > 1e-10) {
        return false;
      }
      trajectory->intervals.push_back(footprint);
    }
    interval_start = interval_end;
  }
  if (trajectory->nodes.size() < 2 ||
      trajectory->intervals.size() + 1 != trajectory->nodes.size()) {
    if (error) *error = "Time mesh contains no moving interval";
    return false;
  }
  for (std::size_t i = 0; i + 1 < trajectory->nodes.size(); ++i) {
    trajectory->nodes[i].w =
        (trajectory->nodes[i + 1].phy - trajectory->nodes[i].phy) /
        trajectory->intervals[i].dt;
    if (std::abs(trajectory->nodes[i].w) >
        config.max_steering_rate + 1e-9) {
      return false;
    }
  }
  trajectory->nodes.back().a = 0.0;
  trajectory->nodes.back().phy = 0.0;
  trajectory->nodes.back().w = 0.0;
  return true;
}

}  // namespace

bool BuildInitialGuess(const planning::HybridAStartResult& hybrid_path,
                       const VehicleParameter& vehicle,
                       const InitialGuessConfig& config,
                       Trajectory* trajectory, std::string* error) {
  if (!trajectory || hybrid_path.x.size() < 2 ||
      hybrid_path.x.size() != hybrid_path.y.size() ||
      hybrid_path.x.size() != hybrid_path.phi.size() ||
      hybrid_path.x.size() != hybrid_path.gear.size()) {
    if (error) *error = "Hybrid A* result arrays are inconsistent";
    return false;
  }
  if (config.min_dt <= 0.0 || config.max_dt < config.min_dt ||
      config.eta <= 0.0 || config.eta > 1.0 ||
      config.max_acceleration <= 0.0 || config.max_steering_rate <= 0.0) {
    if (error) *error = "Invalid temporal mesh configuration";
    return false;
  }

  std::vector<PathSample> path;
  path.reserve(hybrid_path.x.size());
  for (std::size_t i = 0; i < hybrid_path.x.size(); ++i) {
    PathSample sample{hybrid_path.x[i], hybrid_path.y[i],
                      hybrid_path.phi[i], hybrid_path.gear[i]};
    if (!path.empty()) {
      while (sample.theta - path.back().theta > M_PI)
        sample.theta -= 2.0 * M_PI;
      while (sample.theta - path.back().theta < -M_PI)
        sample.theta += 2.0 * M_PI;
      const bool same_position =
          std::hypot(sample.x - path.back().x,
                     sample.y - path.back().y) < 1e-10;
      if (same_position && sample.gear == path.back().gear &&
          i + 1 < hybrid_path.x.size()) {
        continue;
      }
    }
    path.push_back(sample);
  }
  if (path.size() < 2) {
    if (error) *error = "Hybrid A* path has no moving interval";
    return false;
  }

  std::vector<double> path_distance(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i) {
    path_distance[i] = path_distance[i - 1] +
        std::hypot(path[i].x - path[i - 1].x,
                   path[i].y - path[i - 1].y);
  }

  for (double dt = config.max_dt;
       dt + 1e-12 >= config.min_dt;
       dt = std::max(config.min_dt, dt * 0.8)) {
    if (BuildWithStep(path, path_distance, vehicle, config, dt, trajectory,
                      error)) {
      return true;
    }
    if (dt <= config.min_dt + 1e-12) break;
  }
  if (error && error->empty()) {
    *error = "No time step satisfies footprint and steering-rate constraints";
  }
  return false;
}

}  // namespace optimizer
