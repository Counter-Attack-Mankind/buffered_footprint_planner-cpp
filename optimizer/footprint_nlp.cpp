#include "optimizer/footprint_nlp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <casadi/casadi.hpp>

#include "common/math/math_utils.h"
#include "optimizer/embodied_footprint.h"

namespace optimizer {
namespace {

struct HalfspaceObstacle {
  std::array<std::array<double, 2>, 4> a{};
  std::array<double, 4> b{};
  common::math::Vec2d center;
};

common::math::Vec2d ClosestPointOnSegment(
    const common::math::Vec2d& point, const common::math::Vec2d& first,
    const common::math::Vec2d& second) {
  const double dx = second.x() - first.x();
  const double dy = second.y() - first.y();
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1e-20) return first;
  const double ratio = std::clamp(
      ((point.x() - first.x()) * dx + (point.y() - first.y()) * dy) /
          length_squared,
      0.0, 1.0);
  return {first.x() + ratio * dx, first.y() + ratio * dy};
}

double ClosestPolygonPoints(
    const std::array<common::math::Vec2d, 4>& body,
    const common::math::Polygon2d& obstacle,
    common::math::Vec2d* body_point, common::math::Vec2d* obstacle_point) {
  double best = std::numeric_limits<double>::infinity();
  const auto& points = obstacle.points();
  auto consider = [&](const common::math::Vec2d& first,
                      const common::math::Vec2d& second) {
    const double distance = first.DistanceTo(second);
    if (distance < best) {
      best = distance;
      *body_point = first;
      *obstacle_point = second;
    }
  };
  for (const auto& corner : body) {
    for (std::size_t edge = 0; edge < points.size(); ++edge) {
      consider(corner, ClosestPointOnSegment(
                           corner, points[edge], points[(edge + 1) % points.size()]));
    }
  }
  for (const auto& point : points) {
    for (std::size_t edge = 0; edge < body.size(); ++edge) {
      const auto closest = ClosestPointOnSegment(
          point, body[edge], body[(edge + 1) % body.size()]);
      consider(closest, point);
    }
  }
  return best;
}

std::array<double, 4> RepresentDirection(
    const HalfspaceObstacle& obstacle, const common::math::Vec2d& boundary_point,
    double qx, double qy) {
  std::array<double, 4> best{};
  double best_score = std::numeric_limits<double>::infinity();
  auto score = [&](const std::array<double, 4>& candidate) {
    double represented_x = 0.0, represented_y = 0.0, support = 0.0;
    for (std::size_t edge = 0; edge < 4; ++edge) {
      represented_x += obstacle.a[edge][0] * candidate[edge];
      represented_y += obstacle.a[edge][1] * candidate[edge];
      support += obstacle.b[edge] * candidate[edge];
    }
    const double residual = std::hypot(represented_x - qx,
                                       represented_y - qy);
    const double support_error = std::abs(
        support - qx * boundary_point.x() - qy * boundary_point.y());
    const double current_score = support_error + 1000.0 * residual;
    if (current_score < best_score) {
      best_score = current_score;
      best = candidate;
    }
  };
  for (std::size_t edge = 0; edge < 4; ++edge) {
    std::array<double, 4> candidate{};
    candidate[edge] = std::max(
        0.0, obstacle.a[edge][0] * qx + obstacle.a[edge][1] * qy);
    score(candidate);
  }
  for (std::size_t first = 0; first < 4; ++first) {
    for (std::size_t second = first + 1; second < 4; ++second) {
      const double determinant =
          obstacle.a[first][0] * obstacle.a[second][1] -
          obstacle.a[second][0] * obstacle.a[first][1];
      if (std::abs(determinant) <= 1e-12) continue;
      const double first_coefficient =
          (qx * obstacle.a[second][1] - qy * obstacle.a[second][0]) /
          determinant;
      const double second_coefficient =
          (obstacle.a[first][0] * qy - obstacle.a[first][1] * qx) /
          determinant;
      if (first_coefficient < -1e-10 || second_coefficient < -1e-10) continue;
      std::array<double, 4> candidate{};
      candidate[first] = std::max(0.0, first_coefficient);
      candidate[second] = std::max(0.0, second_coefficient);
      score(candidate);
    }
  }
  return best;
}

std::vector<HalfspaceObstacle> BuildHalfspaces(
    const std::vector<common::math::Polygon2d>& obstacles) {
  std::vector<HalfspaceObstacle> result;
  result.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    HalfspaceObstacle halfspace;
    halfspace.center = obstacle.center();
    const auto& points = obstacle.points();
    for (std::size_t edge = 0; edge < 4; ++edge) {
      const auto& first = points[edge];
      const auto& second = points[(edge + 1) % points.size()];
      double nx = second.y() - first.y();
      double ny = -(second.x() - first.x());
      const double length = std::hypot(nx, ny);
      nx /= length;
      ny /= length;
      double boundary = nx * first.x() + ny * first.y();
      if (nx * obstacle.center().x() + ny * obstacle.center().y() > boundary) {
        nx = -nx;
        ny = -ny;
        boundary = -boundary;
      }
      halfspace.a[edge] = {nx, ny};
      halfspace.b[edge] = boundary;
    }
    result.push_back(halfspace);
  }
  return result;
}

std::string GenericToString(const casadi::GenericType& value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

double PositiveViolation(double value) { return std::max(0.0, value); }

void RecordMaximum(double value, const std::string& location,
                   double* maximum, std::string* maximum_location) {
  if (value > *maximum) {
    *maximum = value;
    *maximum_location = location;
  }
}

}  // namespace

OptimizationResult SolveFootprintNlp(
    const Trajectory& initial, const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds,
    const FootprintNlpConfig& config) {
  OptimizationResult result;
  result.initial_guess_succeeded = true;
  result.configured_max_iterations = config.max_iterations;
  result.configured_max_cpu_time = config.max_cpu_time;
  result.configured_tolerance = config.tolerance;
  result.configured_linear_solver = config.linear_solver;
  result.configured_hessian_approximation = "limited-memory";
  result.configured_mu_strategy = "adaptive";
  if (initial.nodes.size() < 2 ||
      initial.intervals.size() + 1 != initial.nodes.size()) {
    result.status = "Invalid initial trajectory dimensions";
    return result;
  }
  std::string geometry_error;
  if (!CheckVehicleGeometryCondition(vehicle, &geometry_error)) {
    result.status = geometry_error;
    return result;
  }

  using casadi::DM;
  using casadi::MX;
  const std::size_t node_count = initial.nodes.size();
  const std::size_t interval_count = node_count - 1;
  const std::size_t obstacle_count = obstacles.size();
  const double front = vehicle.wheel_base + vehicle.front_hang;
  const double rear = vehicle.rear_hang;
  const double half_width = vehicle.width / 2.0;
  const auto halfspaces = BuildHalfspaces(obstacles);

  casadi::Opti opti;
  MX state = opti.variable(7, node_count);
  MX dt = opti.variable(interval_count);
  MX lambda = opti.variable(4, interval_count * obstacle_count);
  MX mu = opti.variable(4, interval_count * obstacle_count);

  opti.subject_to(dt >= config.min_dt);
  opti.subject_to(dt <= config.max_dt);
  opti.subject_to(MX::sum1(dt) <= config.total_time_limit);
  opti.subject_to(lambda >= 0.0);
  opti.subject_to(mu >= 0.0);

  MX objective = MX::sumsqr(dt);
  opti.minimize(objective);

  opti.subject_to(state(0, 0) == initial.nodes.front().x);
  opti.subject_to(state(1, 0) == initial.nodes.front().y);
  opti.subject_to(state(2, 0) == initial.nodes.front().theta);
  opti.subject_to(state(3, 0) == 0.0);
  opti.subject_to(state(4, 0) == 0.0);
  opti.subject_to(state(5, 0) == 0.0);
  opti.subject_to(state(6, 0) == 0.0);
  const std::size_t last = node_count - 1;
  opti.subject_to(state(0, last) == initial.nodes.back().x);
  opti.subject_to(state(1, last) == initial.nodes.back().y);
  opti.subject_to(state(2, last) == initial.nodes.back().theta);
  opti.subject_to(state(3, last) == 0.0);
  opti.subject_to(state(4, last) == 0.0);
  opti.subject_to(state(5, last) == 0.0);
  opti.subject_to(state(6, last) == 0.0);

  opti.subject_to(state(3, casadi::Slice()) <= vehicle.v_max);
  opti.subject_to(state(3, casadi::Slice()) >= -vehicle.v_max);
  opti.subject_to(state(4, casadi::Slice()) <= vehicle.a_max);
  opti.subject_to(state(4, casadi::Slice()) >= -vehicle.a_max);
  opti.subject_to(state(5, casadi::Slice()) <= vehicle.delta_max);
  opti.subject_to(state(5, casadi::Slice()) >= -vehicle.delta_max);
  opti.subject_to(state(6, casadi::Slice()) <= vehicle.omega_max);
  opti.subject_to(state(6, casadi::Slice()) >= -vehicle.omega_max);

  for (std::size_t i = 0; i < interval_count; ++i) {
    const MX x = state(0, i);
    const MX y = state(1, i);
    const MX theta = state(2, i);
    const MX v = state(3, i);
    const MX a = state(4, i);
    const MX phy = state(5, i);
    const MX w = state(6, i);
    const MX interval_dt = dt(i);
    opti.subject_to(state(0, i + 1) == x + v * interval_dt * cos(theta));
    opti.subject_to(state(1, i + 1) == y + v * interval_dt * sin(theta));
    opti.subject_to(state(3, i + 1) == v + interval_dt * a);
    opti.subject_to(state(2, i + 1) ==
                    theta + interval_dt * v * tan(phy) / vehicle.wheel_base);
    opti.subject_to(state(5, i + 1) == phy + interval_dt * w);

    if (i > 0 && initial.intervals[i - 1].gear != initial.intervals[i].gear) {
      opti.subject_to(v == 0.0);
    } else if (initial.intervals[i].gear) {
      opti.subject_to(v >= 0.0);
    } else {
      opti.subject_to(v <= 0.0);
    }

    const MX k = tan(phy) / vehicle.wheel_base;
    const MX abs_k = fabs(k);
    const MX signed_distance = v * interval_dt;
    const MX splus = fmax(signed_distance, 0.0);
    const MX sminus = fmax(-signed_distance, 0.0);
    const MX up = splus + half_width * abs_k * splus;
    const MX down = sminus + half_width * abs_k * sminus;
    const MX left =
        fmax(-rear * k * splus, (front + 0.5 * splus) * k * splus) +
        fmax(-front * k * sminus, (rear + 0.5 * sminus) * k * sminus);
    const MX right =
        fmax(rear * k * splus, -(front + 0.5 * splus) * k * splus) +
        fmax(front * k * sminus, -(rear + 0.5 * sminus) * k * sminus);

    opti.subject_to(abs_k * (splus + sminus) <= M_PI_2);
    opti.subject_to((1.0 + half_width * abs_k) * tan(abs_k * splus) <=
                    rear * abs_k);
    opti.subject_to(abs_k * front * tan(abs_k * splus) <=
                    1.0 + half_width * abs_k);
    opti.subject_to((1.0 + half_width * abs_k) * tan(abs_k * sminus) <=
                    front * abs_k);
    opti.subject_to(abs_k * rear * tan(abs_k * sminus) <=
                    1.0 + half_width * abs_k);

    const MX c = cos(theta);
    const MX sn = sin(theta);
    std::array<MX, 4> corner_x{
        x + (front + up) * c - (half_width + left) * sn,
        x + (front + up) * c + (half_width + right) * sn,
        x - (rear + down) * c + (half_width + right) * sn,
        x - (rear + down) * c - (half_width + left) * sn};
    std::array<MX, 4> corner_y{
        y + (front + up) * sn + (half_width + left) * c,
        y + (front + up) * sn - (half_width + right) * c,
        y - (rear + down) * sn - (half_width + right) * c,
        y - (rear + down) * sn + (half_width + left) * c};
    for (std::size_t corner = 0; corner < 4; ++corner) {
      opti.subject_to(corner_x[corner] >= bounds[0]);
      opti.subject_to(corner_x[corner] <= bounds[1]);
      opti.subject_to(corner_y[corner] >= bounds[2]);
      opti.subject_to(corner_y[corner] <= bounds[3]);
    }

    for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
      const std::size_t column = i * obstacle_count + obstacle;
      MX qx = 0.0;
      MX qy = 0.0;
      MX certificate = 0.0;
      for (std::size_t edge = 0; edge < 4; ++edge) {
        qx += halfspaces[obstacle].a[edge][0] * lambda(edge, column);
        qy += halfspaces[obstacle].a[edge][1] * lambda(edge, column);
        certificate +=
            (halfspaces[obstacle].a[edge][0] * x +
             halfspaces[obstacle].a[edge][1] * y -
             halfspaces[obstacle].b[edge]) * lambda(edge, column);
      }
      certificate -= (front + up) * mu(0, column) +
                     (rear + down) * mu(1, column) +
                     (half_width + left) * mu(2, column) +
                     (half_width + right) * mu(3, column);
      opti.subject_to(certificate >= config.dmin);
      opti.subject_to(mu(0, column) - mu(1, column) + c * qx + sn * qy == 0.0);
      opti.subject_to(mu(2, column) - mu(3, column) - sn * qx + c * qy == 0.0);
      opti.subject_to(qx * qx + qy * qy <= 1.0);
    }
  }

  DM state_initial = DM::zeros(7, node_count);
  DM dt_initial = DM::zeros(interval_count);
  for (std::size_t i = 0; i < node_count; ++i) {
    state_initial(0, i) = initial.nodes[i].x;
    state_initial(1, i) = initial.nodes[i].y;
    state_initial(2, i) = initial.nodes[i].theta;
    state_initial(3, i) = initial.nodes[i].v;
    state_initial(4, i) = initial.nodes[i].a;
    state_initial(5, i) = initial.nodes[i].phy;
    state_initial(6, i) = initial.nodes[i].w;
    if (i < interval_count) dt_initial(i) = initial.intervals[i].dt;
  }
  opti.set_initial(state, state_initial);
  opti.set_initial(dt, dt_initial);

  DM lambda_initial = DM::zeros(4, interval_count * obstacle_count);
  DM mu_initial = DM::zeros(4, interval_count * obstacle_count);
  for (std::size_t i = 0; i < interval_count; ++i) {
    const double c = std::cos(initial.nodes[i].theta);
    const double s = std::sin(initial.nodes[i].theta);
    const auto initial_footprint = ComputeEmbodiedFootprint(
        initial.nodes[i], initial.intervals[i].dt, vehicle);
    for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
      const std::size_t column = i * obstacle_count + obstacle;
      common::math::Vec2d body_point;
      common::math::Vec2d obstacle_point;
      const double distance = ClosestPolygonPoints(
          initial_footprint.corners, obstacles[obstacle],
          &body_point, &obstacle_point);
      double qx = body_point.x() - obstacle_point.x();
      double qy = body_point.y() - obstacle_point.y();
      double norm = std::hypot(qx, qy);
      if (norm > 1e-9) { qx /= norm; qy /= norm; } else { qx = 1.0; qy = 0.0; }
      auto lambda_values = RepresentDirection(
          halfspaces[obstacle], obstacle_point, qx, qy);
      const double longitudinal = c * qx + s * qy;
      const double lateral = -s * qx + c * qy;
      std::array<double, 4> mu_values{{
          std::max(-longitudinal, 0.0), std::max(longitudinal, 0.0),
          std::max(-lateral, 0.0), std::max(lateral, 0.0)}};
      const double rho = distance > 0.0
                             ? std::min(1.0, std::max(
                                   0.25, (config.dmin + 1e-3) / distance))
                             : 1.0;
      for (std::size_t edge = 0; edge < 4; ++edge) {
        lambda_initial(edge, column) = rho * lambda_values[edge];
        mu_initial(edge, column) = rho * mu_values[edge];
      }
    }
  }
  opti.set_initial(lambda, lambda_initial);
  opti.set_initial(mu, mu_initial);

  casadi::Dict solver_options;
  solver_options["max_iter"] = config.max_iterations;
  solver_options["max_cpu_time"] = config.max_cpu_time;
  solver_options["tol"] = config.tolerance;
  solver_options["linear_solver"] = config.linear_solver;
  solver_options["hessian_approximation"] = "limited-memory";
  solver_options["mu_strategy"] = "adaptive";
  solver_options["print_level"] = 0;
  casadi::Dict plugin_options;
  plugin_options["expand"] = true;
  plugin_options["print_time"] = false;
  opti.solver("ipopt", plugin_options, solver_options);

  const auto begin = std::chrono::steady_clock::now();
  try {
    const casadi::OptiSol solution = opti.solve();
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
    const auto stats = solution.stats();
    result.status = GenericToString(stats.at("return_status"));
    result.iterations = static_cast<int>(stats.at("iter_count"));
    result.solver_succeeded = result.status.find("Solve_Succeeded") != std::string::npos ||
                              result.status.find("Solved_To_Acceptable_Level") != std::string::npos;
    result.objective = static_cast<double>(solution.value(objective));
    const DM state_value = solution.value(state);
    const DM dt_value = solution.value(dt);
    const DM lambda_value = solution.value(lambda);
    const DM mu_value = solution.value(mu);
    result.trajectory.nodes.resize(node_count);
    result.trajectory.intervals.resize(interval_count);
    for (std::size_t i = 0; i < node_count; ++i) {
      auto& node = result.trajectory.nodes[i];
      node.x = static_cast<double>(state_value(0, i));
      node.y = static_cast<double>(state_value(1, i));
      node.theta = static_cast<double>(state_value(2, i));
      node.v = static_cast<double>(state_value(3, i));
      node.a = static_cast<double>(state_value(4, i));
      node.phy = static_cast<double>(state_value(5, i));
      node.w = static_cast<double>(state_value(6, i));
      const std::array<std::pair<const char*, double>, 4> variable_violations{{
          {"v", PositiveViolation(std::abs(node.v) - vehicle.v_max)},
          {"a", PositiveViolation(std::abs(node.a) - vehicle.a_max)},
          {"phy", PositiveViolation(std::abs(node.phy) - vehicle.delta_max)},
          {"w", PositiveViolation(std::abs(node.w) - vehicle.omega_max)}}};
      result.variable_bound_check_count += variable_violations.size();
      for (const auto& violation : variable_violations) {
        RecordMaximum(violation.second,
                      "node=" + std::to_string(i) + ",variable=" + violation.first,
                      &result.max_variable_bound_violation,
                      &result.max_variable_bound_location);
      }
    }
    for (std::size_t i = 0; i < interval_count; ++i) {
      const double interval_dt = static_cast<double>(dt_value(i));
      auto footprint = ComputeEmbodiedFootprint(
          result.trajectory.nodes[i], interval_dt, vehicle);
      footprint.gear = initial.intervals[i].gear;
      result.trajectory.intervals[i] = footprint;
      const auto footprint_validity = EvaluateFootprintValidity(footprint, vehicle);
      result.footprint_validity_check_count += 5;
      RecordMaximum(footprint_validity.max_violation(),
                    "interval=" + std::to_string(i),
                    &result.max_footprint_validity_violation,
                    &result.max_footprint_validity_location);
      const auto& first = result.trajectory.nodes[i];
      const auto& second = result.trajectory.nodes[i + 1];
      const std::array<double, 5> dynamics{{
          std::abs(second.x - first.x - first.v * interval_dt * std::cos(first.theta)),
          std::abs(second.y - first.y - first.v * interval_dt * std::sin(first.theta)),
          std::abs(second.v - first.v - interval_dt * first.a),
          std::abs(second.theta - first.theta - interval_dt * first.v *
                       std::tan(first.phy) / vehicle.wheel_base),
          std::abs(second.phy - first.phy - interval_dt * first.w)}};
      result.dynamics_check_count += dynamics.size();
      for (std::size_t equation = 0; equation < dynamics.size(); ++equation) {
        RecordMaximum(dynamics[equation],
                      "interval=" + std::to_string(i) +
                          ",equation=" + std::to_string(equation),
                      &result.max_dynamics_residual,
                      &result.max_dynamics_location);
      }
      for (std::size_t corner_index = 0;
           corner_index < footprint.corners.size(); ++corner_index) {
        const auto& corner = footprint.corners[corner_index];
        const std::array<double, 4> boundary_violations{{
            PositiveViolation(bounds[0] - corner.x()),
            PositiveViolation(corner.x() - bounds[1]),
            PositiveViolation(bounds[2] - corner.y()),
            PositiveViolation(corner.y() - bounds[3])}};
        result.map_boundary_check_count += boundary_violations.size();
        for (std::size_t side = 0; side < boundary_violations.size(); ++side) {
          RecordMaximum(boundary_violations[side],
                        "interval=" + std::to_string(i) +
                            ",corner=" + std::to_string(corner_index) +
                            ",side=" + std::to_string(side),
                        &result.max_map_boundary_violation,
                        &result.max_map_boundary_location);
        }
      }
      for (std::size_t obstacle = 0; obstacle < obstacle_count; ++obstacle) {
        const std::size_t column = i * obstacle_count + obstacle;
        double qx = 0.0, qy = 0.0, certificate = 0.0;
        for (std::size_t edge = 0; edge < 4; ++edge) {
          const double lam = static_cast<double>(lambda_value(edge, column));
          qx += halfspaces[obstacle].a[edge][0] * lam;
          qy += halfspaces[obstacle].a[edge][1] * lam;
          certificate +=
              (halfspaces[obstacle].a[edge][0] * first.x +
               halfspaces[obstacle].a[edge][1] * first.y -
               halfspaces[obstacle].b[edge]) * lam;
        }
        certificate -= (front + footprint.up) * static_cast<double>(mu_value(0, column)) +
                       (rear + footprint.down) * static_cast<double>(mu_value(1, column)) +
                       (half_width + footprint.left) * static_cast<double>(mu_value(2, column)) +
                       (half_width + footprint.right) * static_cast<double>(mu_value(3, column));
        result.obca_check_count += 4;
        RecordMaximum(PositiveViolation(config.dmin - certificate),
                      "interval=" + std::to_string(i) + ",obstacle=" +
                          std::to_string(obstacle) + ",constraint=distance",
                      &result.max_obca_violation, &result.max_obca_location);
        RecordMaximum(PositiveViolation(qx * qx + qy * qy - 1.0),
                      "interval=" + std::to_string(i) + ",obstacle=" +
                          std::to_string(obstacle) + ",constraint=norm",
                      &result.max_obca_violation, &result.max_obca_location);
        const double longitudinal = std::cos(first.theta) * qx +
                                    std::sin(first.theta) * qy;
        const double lateral = -std::sin(first.theta) * qx +
                               std::cos(first.theta) * qy;
        RecordMaximum(std::abs(static_cast<double>(mu_value(0, column)) -
                               static_cast<double>(mu_value(1, column)) + longitudinal),
                      "interval=" + std::to_string(i) + ",obstacle=" +
                          std::to_string(obstacle) + ",constraint=dual_x",
                      &result.max_obca_violation, &result.max_obca_location);
        RecordMaximum(std::abs(static_cast<double>(mu_value(2, column)) -
                               static_cast<double>(mu_value(3, column)) + lateral),
                      "interval=" + std::to_string(i) + ",obstacle=" +
                          std::to_string(obstacle) + ",constraint=dual_y",
                      &result.max_obca_violation, &result.max_obca_location);
      }
    }
    const double residual_limit = std::max(1e-4, 10.0 * config.tolerance);
    result.success = result.solver_succeeded &&
                     result.max_dynamics_residual <= residual_limit &&
                     result.max_variable_bound_violation <= residual_limit &&
                     result.max_footprint_validity_violation <= residual_limit &&
                     result.max_obca_violation <= residual_limit &&
                     result.max_map_boundary_violation <= residual_limit;
  } catch (const std::exception& exception) {
    result.solve_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
    result.status = exception.what();
  }
  return result;
}

bool SaveOptimizationResults(const std::string& result_directory,
                             const Trajectory& initial,
                             const OptimizationResult& result,
                             std::string* error) {
  namespace fs = std::filesystem;
  std::error_code code;
  fs::create_directories(result_directory, code);
  if (code) {
    if (error) *error = code.message();
    return false;
  }
  auto write_trajectory = [&](const fs::path& path, const Trajectory& trajectory) {
    std::ofstream stream(path);
    if (!stream) return false;
    stream << std::setprecision(12)
           << "index,time_s,x_m,y_m,theta_rad,v_mps,a_mps2,phy_rad,w_radps,dt_s,gear\n";
    double time = 0.0;
    for (std::size_t i = 0; i < trajectory.nodes.size(); ++i) {
      const auto& node = trajectory.nodes[i];
      stream << i << ',' << time << ',' << node.x << ',' << node.y << ',' << node.theta << ','
             << node.v << ',' << node.a << ',' << node.phy << ',' << node.w;
      if (i < trajectory.intervals.size()) {
        stream << ',' << trajectory.intervals[i].dt << ','
               << (trajectory.intervals[i].gear ? 1 : -1);
      } else {
        stream << ",0,0";
      }
      stream << '\n';
      if (i < trajectory.intervals.size()) time += trajectory.intervals[i].dt;
    }
    return true;
  };
  if (!write_trajectory(fs::path(result_directory) / "initial_guess.csv", initial) ||
      !write_trajectory(fs::path(result_directory) / "optimized_trajectory.csv",
                        result.trajectory)) {
    if (error) *error = "Cannot write trajectory CSV";
    return false;
  }
  std::ofstream footprints(fs::path(result_directory) / "embodied_footprints.csv");
  footprints << std::setprecision(12)
             << "interval,gear,dt_s,s_m,k_1pm,up_m,down_m,left_m,right_m,"
                "AX_m,AY_m,BX_m,BY_m,CX_m,CY_m,DX_m,DY_m\n";
  for (std::size_t i = 0; i < result.trajectory.intervals.size(); ++i) {
    const auto& fp = result.trajectory.intervals[i];
    footprints << i << ',' << (fp.gear ? 1 : -1) << ',' << fp.dt << ','
               << fp.s << ',' << fp.k << ',' << fp.up << ',' << fp.down << ','
               << fp.left << ',' << fp.right;
    for (const auto& corner : fp.corners) {
      footprints << ',' << corner.x() << ',' << corner.y();
    }
    footprints << '\n';
  }
  std::ofstream summary(fs::path(result_directory) / "solver_summary.txt");
  summary << std::boolalpha
          << "initial_guess_succeeded=" << result.initial_guess_succeeded << '\n'
          << "solver_succeeded=" << result.solver_succeeded << '\n'
          << "validated_success=" << result.success << '\n'
          << "status=" << result.status << '\n'
          << "iterations=" << result.iterations << '\n'
          << "solve_time_s=" << result.solve_time << '\n'
          << "objective=" << result.objective << '\n'
          << "max_dynamics_residual=" << result.max_dynamics_residual << '\n'
          << "max_variable_bound_violation=" << result.max_variable_bound_violation << '\n'
          << "max_footprint_validity_violation=" << result.max_footprint_validity_violation << '\n'
          << "max_obca_violation=" << result.max_obca_violation << '\n'
          << "max_map_boundary_violation=" << result.max_map_boundary_violation << '\n';
  summary << "dynamics_check_count=" << result.dynamics_check_count << '\n'
          << "max_dynamics_location=" << result.max_dynamics_location << '\n'
          << "variable_bound_check_count=" << result.variable_bound_check_count << '\n'
          << "max_variable_bound_location=" << result.max_variable_bound_location << '\n'
          << "footprint_validity_check_count=" << result.footprint_validity_check_count << '\n'
          << "max_footprint_validity_location=" << result.max_footprint_validity_location << '\n'
          << "obca_check_count=" << result.obca_check_count << '\n'
          << "max_obca_location=" << result.max_obca_location << '\n'
          << "map_boundary_check_count=" << result.map_boundary_check_count << '\n'
          << "max_map_boundary_location=" << result.max_map_boundary_location << '\n'
          << "ipopt_max_iter=" << result.configured_max_iterations << '\n'
          << "ipopt_max_cpu_time_s=" << result.configured_max_cpu_time << '\n'
          << "ipopt_tolerance=" << result.configured_tolerance << '\n'
          << "ipopt_linear_solver=" << result.configured_linear_solver << '\n'
          << "ipopt_hessian_approximation="
          << result.configured_hessian_approximation << '\n'
          << "ipopt_mu_strategy=" << result.configured_mu_strategy << '\n'
          << "matlab_interface=AMPL_IPOPT\n"
          << "cpp_interface=CasADi_Opti_IPOPT\n";
  return static_cast<bool>(footprints) && static_cast<bool>(summary);
}

bool WriteCollisionValidation(
    const std::string& output_file, const Trajectory& trajectory,
    const VehicleParameter& vehicle,
    const std::vector<common::math::Polygon2d>& obstacles,
    const std::array<double, 4>& bounds, std::string* error) {
  std::ofstream stream(output_file);
  if (!stream) {
    if (error) *error = "Cannot create collision validation report";
    return false;
  }
  constexpr double evaluation_dt = 0.01;
  std::size_t frames = 0;
  std::size_t collision_frames = 0;
  std::size_t boundary_violation_frames = 0;
  double minimum_obstacle_distance = std::numeric_limits<double>::infinity();
  double maximum_boundary_violation = 0.0;
  if (trajectory.nodes.size() >= 2 &&
      trajectory.intervals.size() + 1 == trajectory.nodes.size()) {
    for (std::size_t i = 0; i < trajectory.intervals.size(); ++i) {
      const double dt = trajectory.intervals[i].dt;
      const std::size_t subdivisions = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(dt / evaluation_dt)));
      const std::size_t first_sample = i == 0 ? 0 : 1;
      for (std::size_t sample = first_sample; sample <= subdivisions; ++sample) {
        const double ratio = static_cast<double>(sample) /
                             static_cast<double>(subdivisions);
        const auto& first = trajectory.nodes[i];
        const auto& second = trajectory.nodes[i + 1];
        const double x = first.x + ratio * (second.x - first.x);
        const double y = first.y + ratio * (second.y - first.y);
        const double theta = first.theta + ratio *
                             (second.theta - first.theta);
        const auto box = vehicle.GenerateBox(common::math::Pose(x, y, theta));
        const common::math::Polygon2d body(box);
        bool collision = false;
        for (const auto& obstacle : obstacles) {
          const double distance = body.DistanceTo(obstacle);
          minimum_obstacle_distance = std::min(minimum_obstacle_distance,
                                                distance);
          if (body.HasOverlap(obstacle)) collision = true;
        }
        bool outside = false;
        for (const auto& corner : box.corners()) {
          const double violation = std::max({
              0.0, bounds[0] - corner.x(), corner.x() - bounds[1],
              bounds[2] - corner.y(), corner.y() - bounds[3]});
          maximum_boundary_violation = std::max(maximum_boundary_violation,
                                                 violation);
          outside = outside || violation > 0.0;
        }
        ++frames;
        if (collision) ++collision_frames;
        if (outside) ++boundary_violation_frames;
      }
    }
  }
  if (!std::isfinite(minimum_obstacle_distance)) {
    minimum_obstacle_distance = 0.0;
  }
  stream << std::setprecision(12) << std::boolalpha
         << "sampling_frequency_hz=100\n"
         << "sampling_period_s=" << evaluation_dt << '\n'
         << "sample_frames=" << frames << '\n'
         << "collision_frames=" << collision_frames << '\n'
         << "collision_ratio="
         << (frames ? static_cast<double>(collision_frames) / frames : 0.0) << '\n'
         << "minimum_obstacle_distance_m=" << minimum_obstacle_distance << '\n'
         << "boundary_violation_frames=" << boundary_violation_frames << '\n'
         << "maximum_boundary_violation_m=" << maximum_boundary_violation << '\n'
         << "passed=" << (frames > 0 && collision_frames == 0 &&
                           boundary_violation_frames == 0) << '\n'
         << "continuous_time_safety_proof=false\n";
  return static_cast<bool>(stream);
}

}  // namespace optimizer
