#pragma once
#include <array>

struct ParkingPlannerConfig {
  // 网格分辨率 (适配小场地2.8x1.8m和小车0.211m)
  double xy_grid_resolution = 0.05;
  double phi_grid_resolution = 0.1;
  size_t next_node_num = 20;   // 前进、倒车各 10 个转向角
  double step_size = 0.03;    // 更细步长

  // 代价权重
  double traj_forward_penalty = 1.0;
  double traj_back_penalty = 1.5;
  double traj_gear_switch_penalty = 10.0;
  double traj_steer_penalty = 1.0;
  double traj_steer_change_penalty = 5.0;

  // 2D A*启发式
  double grid_a_star_xy_resolution = 0.05;

  double delta_t = 0.5;
  double analytic_expansion_cost = 20.0;
  double dubins_phi_gamma = 1.1;
  size_t explore_node_num = 2000000;

  std::array<double, 4> xy_bounds{};
};
