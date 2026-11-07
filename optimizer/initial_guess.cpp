#include "optimizer/initial_guess.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>

#include "optimizer/embodied_footprint.h"

namespace optimizer {
namespace {

constexpr double kDenseTimeStep = 0.001;
constexpr double kMinimumDirectionLength = 0.05;

struct PathPoint {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  int direction = 1;
};

struct DirectionSegment {
  std::size_t start = 0;
  std::size_t end = 0;
  int direction = 1;
  double length = 0.0;
};

struct DenseProfile {
  std::vector<TrajectoryNode> nodes;
  double duration = 0.0;
};

double PathLength(const std::vector<PathPoint>& path, std::size_t start,
                  std::size_t end) {
  double length = 0.0;
  for (std::size_t i = start + 1; i <= end; ++i) {
    length += std::hypot(path[i].x - path[i - 1].x,
                         path[i].y - path[i - 1].y);
  }
  return length;
}

std::vector<DirectionSegment> SplitDirections(
    const std::vector<PathPoint>& path) {
  std::vector<DirectionSegment> segments;
  std::size_t start = 0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (path[i].direction != path[start].direction) {
      segments.push_back({start, i - 1, path[start].direction,
                          PathLength(path, start, i - 1)});
      start = i;
    }
  }
  segments.push_back({start, path.size() - 1, path[start].direction,
                      PathLength(path, start, path.size() - 1)});
  return segments;
}

void RecomputeLengths(const std::vector<PathPoint>& path,
                      std::vector<DirectionSegment>* segments) {
  for (auto& segment : *segments) {
    segment.length = PathLength(path, segment.start, segment.end);
  }
}

void MergeShortSegments(const std::vector<PathPoint>& path,
                        std::vector<DirectionSegment>* segments,
                        InitialGuessDiagnostics* diagnostics) {
  bool changed = true;
  while (changed && segments->size() > 1) {
    changed = false;
    for (std::size_t k = 0; k < segments->size(); ++k) {
      if ((*segments)[k].length >= kMinimumDirectionLength) continue;
      std::ostringstream note;
      note << "segment=" << k << " start=" << (*segments)[k].start
           << " end=" << (*segments)[k].end
           << " direction=" << (*segments)[k].direction
           << " length_m=" << (*segments)[k].length;
      diagnostics->merged_short_segments.push_back(note.str());
      if (k == 0) {
        (*segments)[1].start = (*segments)[0].start;
        segments->erase(segments->begin());
      } else if (k + 1 == segments->size()) {
        (*segments)[k - 1].end = (*segments)[k].end;
        segments->pop_back();
      } else if ((*segments)[k - 1].direction ==
                 (*segments)[k + 1].direction) {
        (*segments)[k - 1].end = (*segments)[k + 1].end;
        segments->erase(segments->begin() + k,
                        segments->begin() + k + 2);
      } else {
        continue;
      }
      RecomputeLengths(path, segments);
      changed = true;
      break;
    }
  }
}

void Linspace(double first, double last, std::size_t count,
              std::vector<double>* values) {
  values->resize(count);
  if (count <= 1) {
    if (count == 1) (*values)[0] = last;
    return;
  }
  for (std::size_t i = 0; i < count; ++i) {
    (*values)[i] = first + (last - first) *
                               static_cast<double>(i) /
                               static_cast<double>(count - 1);
  }
}

bool SolveMinimumTimeProfile(double length, double max_velocity,
                             double max_acceleration,
                             std::vector<double>* distance,
                             std::vector<double>* velocity,
                             std::vector<double>* acceleration,
                             double* duration, std::string* error) {
  if (length <= 0.0 || max_velocity <= 0.0 || max_acceleration <= 0.0) {
    if (error) *error = "Invalid direction-segment length or vehicle limit";
    return false;
  }
  const double threshold = max_velocity * max_velocity /
                           (2.0 * max_acceleration);
  if (length > 2.0 * threshold) {
    const double cruise_distance = length - 2.0 * threshold;
    const double cruise_time = cruise_distance / max_velocity;
    const double slope_time = max_velocity / max_acceleration;
    *duration = 2.0 * slope_time + cruise_time;
    const std::size_t count = std::max<std::size_t>(2, std::llround(
        *duration / kDenseTimeStep));
    std::vector<double> timeline;
    Linspace(0.0, *duration, count, &timeline);
    distance->clear(); velocity->clear(); acceleration->clear();
    for (double time : timeline) {
      if (time <= slope_time) {
        acceleration->push_back(max_acceleration);
        velocity->push_back(time * max_acceleration);
        distance->push_back(0.5 * max_acceleration * time * time);
      } else if (time <= slope_time + cruise_time) {
        acceleration->push_back(0.0);
        velocity->push_back(max_velocity);
        distance->push_back(threshold + max_velocity * (time - slope_time));
      } else {
        const double local = time - slope_time - cruise_time;
        acceleration->push_back(-max_acceleration);
        velocity->push_back(max_velocity - max_acceleration * local);
        distance->push_back(threshold + max_velocity * cruise_time +
                            max_velocity * local -
                            0.5 * max_acceleration * local * local);
      }
    }
  } else {
    const double half_time = std::sqrt(length / max_acceleration);
    const std::size_t half_count = std::max<std::size_t>(2, std::llround(
        half_time / kDenseTimeStep));
    std::vector<double> half_timeline;
    Linspace(0.0, half_time, half_count, &half_timeline);
    *duration = 2.0 * half_time;
    distance->clear(); velocity->clear(); acceleration->clear();
    const double peak_velocity = half_time * max_acceleration;
    const double half_distance = 0.5 * max_acceleration * half_time * half_time;
    for (double time : half_timeline) {
      acceleration->push_back(max_acceleration);
      velocity->push_back(time * max_acceleration);
      distance->push_back(0.5 * max_acceleration * time * time);
    }
    for (double time : half_timeline) {
      acceleration->push_back(-max_acceleration);
      velocity->push_back(peak_velocity - max_acceleration * time);
      distance->push_back(half_distance + peak_velocity * time -
                          0.5 * max_acceleration * time * time);
    }
  }
  return true;
}

void InterpolateGeometry(const std::vector<PathPoint>& path,
                         std::size_t start, std::size_t end,
                         std::vector<PathPoint>* dense,
                         std::vector<double>* cumulative) {
  dense->clear(); cumulative->clear();
  for (std::size_t i = start + 1; i <= end; ++i) {
    const double segment_length = std::hypot(path[i].x - path[i - 1].x,
                                             path[i].y - path[i - 1].y);
    const std::size_t count = std::max<std::size_t>(2,
        std::llround(segment_length * 1000.0));
    for (std::size_t j = 0; j + 1 < count; ++j) {
      const double ratio = static_cast<double>(j) /
                           static_cast<double>(count - 1);
      dense->push_back({path[i - 1].x + ratio * (path[i].x - path[i - 1].x),
                        path[i - 1].y + ratio * (path[i].y - path[i - 1].y),
                        path[i - 1].theta + ratio *
                            (path[i].theta - path[i - 1].theta),
                        path[start].direction});
    }
  }
  dense->push_back(path[end]);
  cumulative->assign(dense->size(), 0.0);
  for (std::size_t i = 1; i < dense->size(); ++i) {
    (*cumulative)[i] = (*cumulative)[i - 1] +
        std::hypot((*dense)[i].x - (*dense)[i - 1].x,
                   (*dense)[i].y - (*dense)[i - 1].y);
  }
}

bool CalculateTimeStamp(const std::vector<PathPoint>& path,
                        std::size_t start, std::size_t end,
                        int direction, const VehicleParameter& vehicle,
                        DenseProfile* profile, std::string* error) {
  const double length = PathLength(path, start, end);
  std::vector<double> s, v, a;
  if (!SolveMinimumTimeProfile(length, vehicle.v_max, vehicle.a_max,
                               &s, &v, &a, &profile->duration, error)) {
    return false;
  }
  std::vector<PathPoint> geometry;
  std::vector<double> cumulative;
  InterpolateGeometry(path, start, end, &geometry, &cumulative);
  profile->nodes.clear();
  profile->nodes.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    double best_error = std::numeric_limits<double>::infinity();
    std::size_t best = 0;
    for (std::size_t j = 0; j < cumulative.size(); ++j) {
      const double current_error = std::abs(s[i] - cumulative[j]);
      if (current_error <= best_error) {
        best_error = current_error;
        best = j;
      }
    }
    TrajectoryNode node;
    node.x = geometry[best].x;
    node.y = geometry[best].y;
    node.theta = geometry[best].theta;
    node.v = direction * std::abs(v[i]);
    node.a = direction * a[i];
    profile->nodes.push_back(node);
  }
  return true;
}

std::string ViolationNames(const FootprintValidity& validity) {
  std::vector<std::string> names;
  if (validity.arc > 1e-12) names.push_back("EF_arc_bound");
  if (validity.forward_cond1 > 1e-12) names.push_back("EF_forward_cond1");
  if (validity.forward_cond2 > 1e-12) names.push_back("EF_forward_cond2");
  if (validity.reverse_cond1 > 1e-12) names.push_back("EF_reverse_cond1");
  if (validity.reverse_cond2 > 1e-12) names.push_back("EF_reverse_cond2");
  std::ostringstream result;
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (i) result << '|';
    result << names[i];
  }
  return result.str();
}

int IntervalDirection(const std::vector<TrajectoryNode>& dense,
                      std::size_t start, std::size_t end, int fallback) {
  for (std::size_t i = start; i <= end; ++i) {
    if (dense[i].v > 1e-6) return 1;
    if (dense[i].v < -1e-6) return -1;
  }
  return fallback;
}

bool SelectTemporalMesh(const std::vector<TrajectoryNode>& dense,
                        double terminal_time,
                        const std::vector<std::size_t>& mandatory,
                        const VehicleParameter& vehicle,
                        const InitialGuessConfig& config,
                        Trajectory* trajectory,
                        InitialGuessDiagnostics* diagnostics,
                        std::string* error) {
  const double base_dt = terminal_time /
                         static_cast<double>(dense.size() - 1);
  const std::size_t min_step = std::max<std::size_t>(1,
      std::ceil((config.min_dt - 1e-12) / base_dt));
  const std::size_t max_step = static_cast<std::size_t>(
      std::floor((config.max_dt + 1e-12) / base_dt));
  if (max_step < min_step) {
    if (error) *error = "Dense base step is larger than max_dt";
    return false;
  }
  std::vector<std::size_t> selected{0};
  std::vector<double> interval_time;
  std::size_t current = 0;
  std::size_t interval_index = 0;
  int previous_direction = dense.front().v < 0.0 ? -1 : 1;
  while (current + 1 < dense.size()) {
    std::size_t boundary = dense.size() - 1;
    for (std::size_t index : mandatory) {
      if (index > current) { boundary = index; break; }
    }
    const std::size_t to_boundary = boundary - current;
    const std::size_t upper = std::min(max_step, to_boundary);
    std::vector<std::size_t> candidates;
    for (std::size_t step = min_step; step <= upper; ++step) {
      const std::size_t remaining = to_boundary - step;
      if (remaining == 0 || remaining >= min_step) candidates.push_back(step);
    }
    if (candidates.empty()) {
      std::ostringstream message;
      message << "No candidate step satisfies mandatory-boundary structure at dense index "
              << current;
      if (error) *error = message.str();
      return false;
    }
    std::size_t last_valid = 0;
    for (std::size_t step : candidates) {
      const std::size_t candidate = current + step;
      const double dt = step * base_dt;
      const int direction = IntervalDirection(dense, current, candidate,
                                              previous_direction);
      bool mixed_direction = false;
      for (std::size_t i = current; i <= candidate; ++i) {
        if ((dense[i].v > 1e-6 && direction < 0) ||
            (dense[i].v < -1e-6 && direction > 0)) mixed_direction = true;
      }
      const auto footprint = ComputeEmbodiedFootprint(dense[current], dt,
                                                       vehicle);
      const auto validity = EvaluateFootprintValidity(footprint, vehicle,
                                                       config.eta);
      if (mixed_direction || validity.max_violation() > 1e-12) {
        diagnostics->rejected_intervals.push_back({
            interval_index, current, candidate, dt, footprint.k, footprint.s,
            mixed_direction ? "mixed_forward_reverse" : ViolationNames(validity),
            validity.max_violation(),
            {{validity.arc, validity.forward_cond1, validity.forward_cond2,
              validity.reverse_cond1, validity.reverse_cond2}}});
        break;
      }
      last_valid = step;
    }
    if (last_valid == 0) {
      std::ostringstream message;
      message << "No admissible interval at dense index " << current;
      if (error) *error = message.str();
      return false;
    }
    current += last_valid;
    selected.push_back(current);
    interval_time.push_back(last_valid * base_dt);
    previous_direction = IntervalDirection(dense, selected[selected.size() - 2],
                                           current, previous_direction);
    ++interval_index;
  }
  trajectory->nodes.clear();
  trajectory->intervals.clear();
  for (std::size_t index : selected) trajectory->nodes.push_back(dense[index]);
  for (std::size_t i = 0; i < interval_time.size(); ++i) {
    const int direction = IntervalDirection(
        dense, selected[i], selected[i + 1], previous_direction);
    auto footprint = ComputeEmbodiedFootprint(trajectory->nodes[i],
                                              interval_time[i], vehicle);
    footprint.gear = direction > 0;
    trajectory->intervals.push_back(footprint);
  }
  return true;
}

void FinalizeInitialValues(const VehicleParameter& vehicle,
                           const InitialGuessConfig& config,
                           Trajectory* trajectory,
                           InitialGuessDiagnostics* diagnostics) {
  for (std::size_t i = 0; i < trajectory->intervals.size(); ++i) {
    const double dt = trajectory->intervals[i].dt;
    const double raw_a = (trajectory->nodes[i + 1].v -
                          trajectory->nodes[i].v) / dt;
    const double raw_w = (trajectory->nodes[i + 1].phy -
                          trajectory->nodes[i].phy) / dt;
    diagnostics->max_abs_raw_steering_rate = std::max(
        diagnostics->max_abs_raw_steering_rate, std::abs(raw_w));
    if (std::abs(raw_a) > config.max_acceleration) {
      ++diagnostics->acceleration_clip_count;
    }
    if (std::abs(raw_w) > config.max_steering_rate) {
      ++diagnostics->steering_rate_clip_count;
    }
    trajectory->nodes[i].a = std::clamp(
        raw_a, -config.max_acceleration, config.max_acceleration);
    trajectory->nodes[i].w = std::clamp(
        raw_w, -config.max_steering_rate, config.max_steering_rate);
  }
  trajectory->nodes.front().v = 0.0;
  trajectory->nodes.front().a = 0.0;
  trajectory->nodes.front().phy = 0.0;
  trajectory->nodes.front().w = 0.0;
  trajectory->nodes.back().v = 0.0;
  trajectory->nodes.back().a = 0.0;
  trajectory->nodes.back().phy = 0.0;
  trajectory->nodes.back().w = 0.0;

  diagnostics->min_dt = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < trajectory->intervals.size(); ++i) {
    const double dt = trajectory->intervals[i].dt;
    auto footprint = ComputeEmbodiedFootprint(trajectory->nodes[i], dt, vehicle);
    footprint.gear = trajectory->intervals[i].gear;
    trajectory->intervals[i] = footprint;
    diagnostics->min_dt = std::min(diagnostics->min_dt, dt);
    diagnostics->max_dt = std::max(diagnostics->max_dt, dt);
    diagnostics->total_time += dt;
    diagnostics->max_footprint_violation = std::max(
        diagnostics->max_footprint_violation,
        EvaluateFootprintValidity(footprint, vehicle, config.eta).max_violation());
    const auto& first = trajectory->nodes[i];
    const auto& second = trajectory->nodes[i + 1];
    diagnostics->max_euler_residual = std::max({
        diagnostics->max_euler_residual,
        std::abs(second.x - first.x - first.v * dt * std::cos(first.theta)),
        std::abs(second.y - first.y - first.v * dt * std::sin(first.theta)),
        std::abs(second.theta - first.theta - first.v * dt *
                 std::tan(first.phy) / vehicle.wheel_base),
        std::abs(second.v - first.v - dt * first.a),
        std::abs(second.phy - first.phy - dt * first.w)});
  }
  for (const auto& node : trajectory->nodes) {
    diagnostics->max_abs_velocity = std::max(diagnostics->max_abs_velocity,
                                               std::abs(node.v));
    diagnostics->max_abs_acceleration = std::max(
        diagnostics->max_abs_acceleration, std::abs(node.a));
    diagnostics->max_abs_steering = std::max(diagnostics->max_abs_steering,
                                              std::abs(node.phy));
    diagnostics->max_abs_clipped_steering_rate = std::max(
        diagnostics->max_abs_clipped_steering_rate, std::abs(node.w));
    diagnostics->max_velocity_bound_violation = std::max(
        diagnostics->max_velocity_bound_violation,
        std::max(0.0, std::abs(node.v) - vehicle.v_max));
    diagnostics->max_acceleration_bound_violation = std::max(
        diagnostics->max_acceleration_bound_violation,
        std::max(0.0, std::abs(node.a) - vehicle.a_max));
    diagnostics->max_steering_bound_violation = std::max(
        diagnostics->max_steering_bound_violation,
        std::max(0.0, std::abs(node.phy) - vehicle.delta_max));
    diagnostics->max_steering_rate_bound_violation = std::max(
        diagnostics->max_steering_rate_bound_violation,
        std::max(0.0, std::abs(node.w) - vehicle.omega_max));
    if (!std::isfinite(node.x) || !std::isfinite(node.y) ||
        !std::isfinite(node.theta) || !std::isfinite(node.v) ||
        !std::isfinite(node.a) || !std::isfinite(node.phy) ||
        !std::isfinite(node.w)) {
      ++diagnostics->nonfinite_value_count;
    }
  }
  for (std::size_t i = 0; i < trajectory->intervals.size(); ++i) {
    if ((trajectory->intervals[i].gear && trajectory->nodes[i].v < -1e-9) ||
        (!trajectory->intervals[i].gear && trajectory->nodes[i].v > 1e-9)) {
      ++diagnostics->direction_sign_mismatch_count;
    }
  }
  diagnostics->final_node_count = trajectory->nodes.size();
}

}  // namespace

bool BuildInitialGuess(const planning::HybridAStartResult& hybrid_path,
                       const VehicleParameter& vehicle,
                       const InitialGuessConfig& config,
                       Trajectory* trajectory,
                       InitialGuessDiagnostics* diagnostics,
                       std::string* error) {
  *diagnostics = InitialGuessDiagnostics{};
  diagnostics->hybrid_point_count = hybrid_path.x.size();
  if (!trajectory || hybrid_path.x.size() < 2 ||
      hybrid_path.x.size() != hybrid_path.y.size() ||
      hybrid_path.x.size() != hybrid_path.phi.size() ||
      hybrid_path.x.size() != hybrid_path.gear.size()) {
    diagnostics->failure_reason = "Hybrid A* result arrays are inconsistent";
    if (error) *error = diagnostics->failure_reason;
    return false;
  }

  std::vector<PathPoint> path;
  for (std::size_t i = 0; i < hybrid_path.x.size(); ++i) {
    if (i > 0 && hybrid_path.x[i] == hybrid_path.x[i - 1] &&
        hybrid_path.y[i] == hybrid_path.y[i - 1]) continue;
    int direction = hybrid_path.gear[i] ? 1 : -1;
    if (hybrid_path.v.size() == hybrid_path.x.size()) {
      if (hybrid_path.v[i] > 0.0) direction = 1;
      if (hybrid_path.v[i] < 0.0) direction = -1;
    }
    path.push_back({hybrid_path.x[i], hybrid_path.y[i],
                    hybrid_path.phi[i], direction});
  }
  diagnostics->deduplicated_point_count = path.size();
  if (path.size() < 2) {
    diagnostics->failure_reason = "Path is a single point after deduplication";
    if (error) *error = diagnostics->failure_reason;
    return false;
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    while (path[i].theta - path[i - 1].theta > M_PI) path[i].theta -= 2.0 * M_PI;
    while (path[i].theta - path[i - 1].theta < -M_PI) path[i].theta += 2.0 * M_PI;
  }
  diagnostics->hybrid_path_length = PathLength(path, 0, path.size() - 1);
  auto segments = SplitDirections(path);
  diagnostics->raw_direction_segments = segments.size();
  diagnostics->raw_gear_switches = segments.empty() ? 0 : segments.size() - 1;
  MergeShortSegments(path, &segments, diagnostics);
  diagnostics->effective_direction_segments = segments.size();
  diagnostics->effective_gear_switches = segments.empty() ? 0 : segments.size() - 1;

  std::vector<std::size_t> cuts{0};
  for (std::size_t i = 1; i < segments.size(); ++i) cuts.push_back(segments[i].start);
  cuts.push_back(path.size() - 1);
  std::vector<TrajectoryNode> dense;
  std::vector<std::size_t> mandatory;
  double terminal_time = 0.0;
  for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
    DenseProfile profile;
    if (!CalculateTimeStamp(path, cuts[i], cuts[i + 1],
                            segments[i].direction, vehicle, &profile, error)) {
      diagnostics->failure_reason = error ? *error : "CalculateTimeStamp failed";
      return false;
    }
    if (i > 0 && !profile.nodes.empty()) profile.nodes.erase(profile.nodes.begin());
    dense.insert(dense.end(), profile.nodes.begin(), profile.nodes.end());
    terminal_time += profile.duration;
    if (i + 2 < cuts.size()) mandatory.push_back(dense.size() - 1);
  }
  if (dense.size() < 2) {
    diagnostics->failure_reason = "Dense trajectory has fewer than two nodes";
    if (error) *error = diagnostics->failure_reason;
    return false;
  }
  const double base_dt = terminal_time / static_cast<double>(dense.size() - 1);
  for (std::size_t i = 1; i + 1 < dense.size(); ++i) {
    if (std::abs(dense[i].v) < 1e-6) {
      dense[i].phy = 0.0;
    } else {
      dense[i].phy = std::clamp(
          std::atan((dense[i + 1].theta - dense[i].theta) *
                    vehicle.wheel_base / (base_dt * dense[i].v)),
          -vehicle.delta_max, vehicle.delta_max);
    }
  }
  for (std::size_t i = 1; i + 1 < dense.size(); ++i) {
    dense[i].w = std::clamp((dense[i + 1].phy - dense[i].phy) / base_dt,
                            -vehicle.omega_max, vehicle.omega_max);
  }
  diagnostics->dense_node_count = dense.size();
  if (!SelectTemporalMesh(dense, terminal_time, mandatory, vehicle, config,
                          trajectory, diagnostics, error)) {
    diagnostics->failure_reason = error ? *error : "TimeDistribution failed";
    return false;
  }
  diagnostics->endpoint_pose_error = std::max({
      std::hypot(trajectory->nodes.front().x - dense.front().x,
                 trajectory->nodes.front().y - dense.front().y),
      std::abs(trajectory->nodes.front().theta - dense.front().theta),
      std::hypot(trajectory->nodes.back().x - dense.back().x,
                 trajectory->nodes.back().y - dense.back().y),
      std::abs(trajectory->nodes.back().theta - dense.back().theta)});
  FinalizeInitialValues(vehicle, config, trajectory, diagnostics);
  diagnostics->success = true;
  return true;
}

bool SaveInitialGuessDiagnostics(const std::string& output_file,
                                 const InitialGuessDiagnostics& d,
                                 std::string* error) {
  std::ofstream stream(output_file);
  if (!stream) {
    if (error) *error = "Cannot create initial guess diagnostics";
    return false;
  }
  stream << std::setprecision(12) << std::boolalpha
         << "success=" << d.success << '\n'
         << "failure_reason=" << d.failure_reason << '\n'
         << "hybrid_point_count=" << d.hybrid_point_count << '\n'
         << "deduplicated_point_count=" << d.deduplicated_point_count << '\n'
         << "hybrid_path_length_m=" << d.hybrid_path_length << '\n'
         << "raw_direction_segments=" << d.raw_direction_segments << '\n'
         << "effective_direction_segments=" << d.effective_direction_segments << '\n'
         << "raw_gear_switches=" << d.raw_gear_switches << '\n'
         << "effective_gear_switches=" << d.effective_gear_switches << '\n'
         << "dense_node_count=" << d.dense_node_count << '\n'
         << "final_nlp_node_count=" << d.final_node_count << '\n'
         << "min_dt_s=" << d.min_dt << '\n'
         << "max_dt_s=" << d.max_dt << '\n'
         << "total_time_s=" << d.total_time << '\n'
         << "max_abs_velocity_mps=" << d.max_abs_velocity << '\n'
         << "max_abs_acceleration_mps2=" << d.max_abs_acceleration << '\n'
         << "max_abs_steering_rad=" << d.max_abs_steering << '\n'
         << "max_abs_raw_steering_rate_radps=" << d.max_abs_raw_steering_rate << '\n'
         << "max_abs_clipped_steering_rate_radps=" << d.max_abs_clipped_steering_rate << '\n'
         << "acceleration_clip_count=" << d.acceleration_clip_count << '\n'
         << "steering_rate_clip_count=" << d.steering_rate_clip_count << '\n'
         << "max_euler_residual=" << d.max_euler_residual << '\n'
         << "max_footprint_violation=" << d.max_footprint_violation << '\n';
  stream << "endpoint_pose_error=" << d.endpoint_pose_error << '\n'
         << "max_velocity_bound_violation=" << d.max_velocity_bound_violation << '\n'
         << "max_acceleration_bound_violation=" << d.max_acceleration_bound_violation << '\n'
         << "max_steering_bound_violation=" << d.max_steering_bound_violation << '\n'
         << "max_steering_rate_bound_violation=" << d.max_steering_rate_bound_violation << '\n'
         << "nonfinite_value_count=" << d.nonfinite_value_count << '\n'
         << "direction_sign_mismatch_count=" << d.direction_sign_mismatch_count << '\n';
  for (const auto& item : d.merged_short_segments) {
    stream << "merged_short_segment=" << item << '\n';
  }
  for (const auto& item : d.rejected_intervals) {
    stream << "rejected_candidate="
           << "interval:" << item.interval_index
           << ",dense_start:" << item.dense_start
           << ",dense_end:" << item.dense_end
           << ",dt_s:" << item.dt
           << ",curvature_1pm:" << item.curvature
           << ",signed_distance_m:" << item.signed_distance
           << ",conditions:" << item.violated_conditions
           << ",max_violation:" << item.max_violation
           << ",EF_arc_bound:" << item.violations[0]
           << ",EF_forward_cond1:" << item.violations[1]
           << ",EF_forward_cond2:" << item.violations[2]
           << ",EF_reverse_cond1:" << item.violations[3]
           << ",EF_reverse_cond2:" << item.violations[4] << '\n';
  }
  return static_cast<bool>(stream);
}

}  // namespace optimizer
