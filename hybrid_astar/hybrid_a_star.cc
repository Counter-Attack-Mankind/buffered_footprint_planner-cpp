/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * @file
 */

#include "hybrid_astar/hybrid_a_star.h"

#include <memory>
#include <utility>

#include "common/util/csv_logger.h"

namespace planning {

using common::util::HybridAStarLogger;

using common::math::Box2d;
using common::math::Vec2d;

HybridAStar::HybridAStar(const ParkingPlannerConfig &config, 
                         const VehicleParameter &vehicle, 
                         const std::vector<common::math::Polygon2d> &obstacles)
    : obstacles_(obstacles) {
  planner_open_space_config_ = config;
  vehicle_param_ = vehicle;
  XYbounds_ = config.xy_bounds;
  reed_shepp_generator_ = std::unique_ptr<ReedShepp>(
      new ReedShepp(vehicle_param_, planner_open_space_config_));
  grid_a_star_heuristic_generator_ = std::unique_ptr<GridSearch>(new GridSearch(config, obstacles));
  next_node_num_ =
      planner_open_space_config_.next_node_num;
  max_steer_angle_ =
      vehicle_param_.delta_max / vehicle_param_.steer_ratio;
  step_size_ = planner_open_space_config_.step_size;
  xy_grid_resolution_ =
      planner_open_space_config_.xy_grid_resolution;
  delta_t_ = planner_open_space_config_.delta_t;
  traj_forward_penalty_ =
      planner_open_space_config_.traj_forward_penalty;
  traj_back_penalty_ =
      planner_open_space_config_.traj_back_penalty;
  traj_gear_switch_penalty_ =
      planner_open_space_config_.traj_gear_switch_penalty;
  traj_steer_penalty_ =
      planner_open_space_config_.traj_steer_penalty;
  traj_steer_change_penalty_ = planner_open_space_config_.traj_steer_change_penalty;
}

bool HybridAStar::AnalyticExpansion(const std::shared_ptr<Node3d>& current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    return false;
  }

  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  // 记录Reed-Shepp路径
  HybridAStarLogger::LogReedSheppPath(reeds_shepp_to_check->x,
                                      reeds_shepp_to_check->y,
                                      reeds_shepp_to_check->phi,
                                      reeds_shepp_to_check->gear,
                                      reeds_shepp_to_check->total_length);

  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_);
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(const std::shared_ptr<Node3d>& node) {
  size_t node_step_size = node->GetStepSize();
  const auto& traversed_x = node->GetXs();
  const auto& traversed_y = node->GetYs();
  const auto& traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }

  for (size_t i = check_start_index; i < node_step_size; ++i) {
    common::math::Pose pose(traversed_x[i], traversed_y[i], traversed_phi[i]);
    auto vehicle_box = vehicle_param_.GenerateBox(pose);
    for (const auto& corner : vehicle_box.corners()) {
      if (corner.x() > XYbounds_[1] || corner.x() < XYbounds_[0] ||
          corner.y() > XYbounds_[3] || corner.y() < XYbounds_[2]) {
        return false;
      }
    }
    common::math::Polygon2d vehicle_polygon(vehicle_box);
    for (const auto &ob : obstacles_) {
      if (vehicle_polygon.HasOverlap(ob)) {
        return false;
      }
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath>& reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::make_shared<Node3d>(
      reeds_shepp_to_end->x, reeds_shepp_to_end->y, reeds_shepp_to_end->phi,
      XYbounds_, planner_open_space_config_);
  if (!reeds_shepp_to_end->gear.empty()) {
    end_node->SetDirec(reeds_shepp_to_end->gear.back());
  }
  end_node->SetPre(std::move(current_node));
  close_set_.emplace(end_node->GetIndex(), end_node);
  return end_node;
}

std::shared_ptr<Node3d> HybridAStar::Next_node_generator(
    const std::shared_ptr<Node3d>& current_node, size_t next_node_index) {
  double steering = 0.0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
        static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    size_t index = next_node_index - next_node_num_ / 2;
    steering =
        -max_steer_angle_ +
        (2 * max_steer_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
        static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = common::math::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }
  std::shared_ptr<Node3d> next_node = std::make_shared<Node3d>(
      intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 planner_open_space_config_);
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(const std::shared_ptr<Node3d>& current_node,
                                    const std::shared_ptr<Node3d>& next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(const std::shared_ptr<Node3d>& current_node,
                             const std::shared_ptr<Node3d>& next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(const std::shared_ptr<Node3d>& next_node) {
  return grid_a_star_heuristic_generator_->CheckDpMap(next_node->GetX(),
                                                      next_node->GetY());
}

bool HybridAStar::GetResult(HybridAStartResult* result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      printf("result size check failed");
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      printf("states sizes are not equal");
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;
  result->gear.clear();
  result->gear.reserve(result->x.size());
  for (size_t i = 0; i + 1 < result->x.size(); ++i) {
    const double dx = result->x[i + 1] - result->x[i];
    const double dy = result->y[i + 1] - result->y[i];
    const double projection = dx * std::cos(result->phi[i]) +
                              dy * std::sin(result->phi[i]);
    result->gear.push_back(std::abs(projection) < 1e-9 && !result->gear.empty()
                               ? result->gear.back()
                               : projection >= 0.0);
  }
  result->gear.push_back(result->gear.empty() ? true : result->gear.back());

  if (!GetTemporalProfile(result)) {
    printf("GetSpeedProfile from Hybrid Astar path fails");
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    printf("state sizes not equal");
    return false;
  }
  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    printf("control sizes not equal or not right");
    return false;
  }
  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult* result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    printf("result size check when generating speed and acceleration fail");
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  if (result->gear.size() != x_size) {
    return false;
  }
  result->v.clear();
  result->a.clear();
  result->steer.clear();
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    const double distance = std::hypot(result->x[i + 1] - result->x[i - 1],
                                       result->y[i + 1] - result->y[i - 1]);
    const double discrete_v = (result->gear[i] ? 1.0 : -1.0) *
                              distance / (2.0 * delta_t_);
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = common::math::NormalizeAngle(
                                result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base / step_size_;
    if (result->gear[i]) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
  }
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult& result,
    std::vector<HybridAStartResult>* partitioned_result) {
  const auto& x = result.x;
  const auto& y = result.y;
  const auto& phi = result.phi;
  if (x.size() < 2 || x.size() != y.size() || x.size() != phi.size() ||
      x.size() != result.gear.size()) {
    printf("states sizes are not equal when do trajectory partitioning of Hybrid A Star result");
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto* current_traj = &(partitioned_result->back());
  bool current_gear = result.gear.front();
  current_traj->x.push_back(x.front());
  current_traj->y.push_back(y.front());
  current_traj->phi.push_back(phi.front());
  current_traj->gear.push_back(current_gear);
  for (size_t i = 0; i < horizon - 1; ++i) {
    const bool gear = result.gear[i];
    if (gear != current_gear) {
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      current_traj->gear.push_back(gear);
    }
    current_traj->x.push_back(x[i + 1]);
    current_traj->y.push_back(y[i + 1]);
    current_traj->phi.push_back(phi[i + 1]);
    current_traj->gear.push_back(gear);
  }


  // Retrieve v, a and steer from path
  for (auto& result : *partitioned_result) {
    if (!GenerateSpeedAcceleration(&result)) {
      printf("GenerateSpeedAcceleration fail");
      return false;
    }
  }

  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult* result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    return false;
  }
  HybridAStartResult stitched_result;
  for (const auto& result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.gear.begin(), result.gear.end() - 1,
              std::back_inserter(stitched_result.gear));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.gear.push_back(partitioned_results.back().gear.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  stitched_result.accumulated_s.reserve(stitched_result.x.size());
  stitched_result.accumulated_s.push_back(0.0);
  for (size_t i = 1; i < stitched_result.x.size(); ++i) {
    stitched_result.accumulated_s.push_back(
        stitched_result.accumulated_s.back() +
        std::hypot(stitched_result.x[i] - stitched_result.x[i - 1],
                   stitched_result.y[i] - stitched_result.y[i - 1]));
  }
  *result = stitched_result;
  return true;
}

bool HybridAStar::Plan(
    double sx, double sy, double sphi, double ex, double ey, double ephi, HybridAStartResult* result) {
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  // 初始化CSV记录器
  HybridAStarLogger::Initialize();

  // load nodes and obstacles
  start_node_.reset(
      new Node3d(sx, sy, sphi, XYbounds_, planner_open_space_config_));
  end_node_.reset(
      new Node3d(ex, ey, ephi, XYbounds_, planner_open_space_config_));
  if (!ValidityCheck(start_node_)) {
    printf("start_node in collision with obstacles");
    HybridAStarLogger::Finalize();
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    printf("end_node in collision with obstacles");
    HybridAStarLogger::Finalize();
    return false;
  }
  grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_);

  // DEBUG: check if DP map reaches start
  double start_dp_cost = grid_a_star_heuristic_generator_->CheckDpMap(sx, sy);
  printf("DP map heuristic at start (%.2f, %.2f): %.4f\n", sx, sy, start_dp_cost);
  if (std::isinf(start_dp_cost)) {
    printf("WARNING: 2D grid search cannot reach start from goal! Check obstacles/bounds.\n");
    printf("  Bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]\n",
           XYbounds_[0], XYbounds_[1], XYbounds_[2], XYbounds_[3]);
    printf("  Start: (%.3f, %.3f, %.3f), Goal: (%.3f, %.3f, %.3f)\n",
           sx, sy, sphi, ex, ey, ephi);
  }

  // load open set, pq
  open_set_.emplace(start_node_->GetIndex(), start_node_);
  open_pq_.emplace(start_node_->GetIndex(), start_node_->GetCost());

  // Hybrid A* begins
  size_t explored_node_num = 0;
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  while (!open_pq_.empty()) {
    // CRITICAL FIX: Add exploration limit to prevent infinite search
    if (explored_node_num >= planner_open_space_config_.explore_node_num) {
      printf("Hybrid A* reached exploration limit: %ld nodes explored\n",
             planner_open_space_config_.explore_node_num);
      break;
    }

    // take out the lowest cost neighboring node
    const int64_t current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];

    // OPTIMIZATION: Check distance to goal - try analytic expansion more frequently when close
    double dx = current_node->GetX() - end_node_->GetX();
    double dy = current_node->GetY() - end_node_->GetY();
    double distance_to_goal = std::sqrt(dx * dx + dy * dy);

    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    if (AnalyticExpansion(current_node)) {
      printf("Hybrid A* found solution at distance %.2f from goal\n", distance_to_goal);
      break;
    }
    close_set_.emplace(current_node->GetIndex(), current_node);
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        CalculateNodeCost(current_node, next_node);
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());

        // 记录搜索节点
        HybridAStarLogger::LogSearchNode(
            next_node->GetX(), next_node->GetY(), next_node->GetPhi(),
            next_node->GetTrajCost(), next_node->GetHeuCost(),
            next_node->GetDirec(), next_node->GetSteer(),
            explored_node_num);
      }
    }
  }
  if (final_node_ == nullptr) {
    printf("Hybrid A* search failed: final_node is nullptr\n");
    printf("Total nodes explored: %zu\n", explored_node_num);
    HybridAStarLogger::Finalize();
    return false;
  }
  if (!GetResult(result)) {
    HybridAStarLogger::Finalize();
    return false;
  }

  // Calculate path statistics
  double path_length = 0.0;
  for (size_t i = 1; i < result->x.size(); ++i) {
    double dx = result->x[i] - result->x[i-1];
    double dy = result->y[i] - result->y[i-1];
    path_length += std::sqrt(dx * dx + dy * dy);
  }

  printf("Hybrid A* search succeeded:\n");
  printf("  - Nodes explored: %zu\n", explored_node_num);
  printf("  - Path points: %zu\n", result->x.size());
  printf("  - Path length: %.2f m\n", path_length);

  // 记录最终的粗路径结果
  HybridAStarLogger::LogCoarsePath(result->x, result->y, result->phi,
                                   result->v, result->a, result->steer);

  HybridAStarLogger::Finalize();
  return true;
}
}  // namespace planning
