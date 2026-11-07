#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "common/math/math_utils.h"
#include "environment/scenario_manager.h"
#include "hybrid_astar/hybrid_a_star.h"
#include "hybrid_astar/parking_planner_config.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "visualize/visualization_plot.h"

namespace {

struct Pose2d {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PlanningSnapshot {
  Pose2d start;
  Pose2d goal;
  std::array<double, 4> bounds;
  std::vector<common::math::Polygon2d> obstacles;
  VehicleParameter vehicle;
  ParkingPlannerConfig planner;
};

Pose2d FromPose(const geometry_msgs::Pose& pose) {
  return {pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)};
}

class PlanningNode {
 public:
  PlanningNode() : private_node_("~") {}

  bool Initialize() {
    if (!scenario_.Load(private_node_) || !LoadVehicle() || !LoadPlanner()) {
      return false;
    }
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    VisualizationPlot::Init(private_node_, scenario_.frame_id(),
                            "/buffered_footprint_planner/markers");
    command_client_ = private_node_.serviceClient<
        buffered_footprint_planner::SetObjectPose>(
        "/simulation/set_object_pose");
    obstacle_subscriber_ = private_node_.subscribe(
        "/simulation/scene_obstacle_poses", 1,
        &PlanningNode::ObstacleCallback, this);
    start_subscriber_ = private_node_.subscribe(
        "/simulation/locked_start", 1, &PlanningNode::StartCallback, this);
    lock_subscriber_ = private_node_.subscribe(
        "/simulation/start_locked", 1, &PlanningNode::LockCallback, this);
    revision_subscriber_ = private_node_.subscribe(
        "/simulation/scene_revision", 1,
        &PlanningNode::RevisionCallback, this);
    goal_subscriber_ = private_node_.subscribe(
        "/move_base_simple/goal", 1, &PlanningNode::GoalCallback, this);
    ROS_INFO("Planning node ready; waiting for a locked start and RViz goal");
    return true;
  }

 private:
  bool LoadVehicle() {
    if (!private_node_.getParam("wheel_base", vehicle_.wheel_base) ||
        !private_node_.getParam("front_hang", vehicle_.front_hang) ||
        !private_node_.getParam("rear_hang", vehicle_.rear_hang) ||
        !private_node_.getParam("width", vehicle_.width) ||
        !private_node_.getParam("max_velocity", vehicle_.v_max) ||
        !private_node_.getParam("max_acceleration", vehicle_.a_max) ||
        !private_node_.getParam("max_phi", vehicle_.delta_max) ||
        !private_node_.getParam("max_omega", vehicle_.omega_max)) {
      ROS_ERROR("Missing vehicle parameters in planner.yaml");
      return false;
    }
    vehicle_.steer_ratio = 1.0;
    vehicle_.GenerateDisc();
    return true;
  }

  bool LoadPlanner() {
    private_node_.param("xy_grid_resolution", planner_.xy_grid_resolution, 0.02);
    private_node_.param("phi_grid_resolution", planner_.phi_grid_resolution, 0.05);
    private_node_.param("step_size", planner_.step_size, 0.03);
    private_node_.param("traj_forward_penalty", planner_.traj_forward_penalty, 1.0);
    private_node_.param("traj_back_penalty", planner_.traj_back_penalty, 1.5);
    private_node_.param("traj_gear_switch_penalty", planner_.traj_gear_switch_penalty, 10.0);
    private_node_.param("traj_steer_penalty", planner_.traj_steer_penalty, 1.0);
    private_node_.param("traj_steer_change_penalty", planner_.traj_steer_change_penalty, 5.0);
    private_node_.param("grid_a_star_xy_resolution", planner_.grid_a_star_xy_resolution, 0.03);
    private_node_.param("delta_t", planner_.delta_t, 0.5);
    private_node_.param("analytic_expansion_cost", planner_.analytic_expansion_cost, 20.0);
    private_node_.param("dubins_phi_gamma", planner_.dubins_phi_gamma, 1.1);
    int next_node_num = 20;
    int explore_node_num = 2000000;
    private_node_.param("next_node_num", next_node_num, next_node_num);
    private_node_.param("explore_node_num", explore_node_num, explore_node_num);
    if (next_node_num < 4 || next_node_num % 2 != 0 ||
        explore_node_num <= 0 || planner_.xy_grid_resolution <= 0.0 ||
        planner_.phi_grid_resolution <= 0.0 || planner_.step_size <= 0.0 ||
        planner_.grid_a_star_xy_resolution <= 0.0 || planner_.delta_t <= 0.0) {
      ROS_ERROR("Invalid Hybrid A* parameters");
      return false;
    }
    planner_.next_node_num = static_cast<std::size_t>(next_node_num);
    planner_.explore_node_num = static_cast<std::size_t>(explore_node_num);
    planner_.xy_bounds = scenario_.bounds();
    return true;
  }

  bool ValidVehiclePose(const Pose2d& pose) const {
    const auto box = vehicle_.GenerateBox(
        common::math::Pose(pose.x, pose.y, pose.yaw));
    for (const auto& corner : box.corners()) {
      if (corner.x() < scenario_.bounds()[0] ||
          corner.x() > scenario_.bounds()[1] ||
          corner.y() < scenario_.bounds()[2] ||
          corner.y() > scenario_.bounds()[3]) return false;
    }
    const common::math::Polygon2d footprint(box);
    for (const auto& obstacle : scenario_.obstacles()) {
      if (footprint.HasOverlap(obstacle.polygon())) return false;
    }
    return true;
  }

  void ObstacleCallback(const geometry_msgs::PoseArray::ConstPtr& message) {
    if (message->poses.size() != scenario_.obstacles().size()) return;
    for (std::size_t i = 0; i < message->poses.size(); ++i) {
      const auto pose = FromPose(message->poses[i]);
      scenario_.SetObstaclePose(scenario_.obstacles()[i].id,
                                pose.x, pose.y, pose.yaw);
    }
  }

  void StartCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    locked_start_ = FromPose(message->pose);
  }

  void LockCallback(const std_msgs::Bool::ConstPtr& message) {
    start_locked_ = message->data;
  }

  void RevisionCallback(const std_msgs::UInt64::ConstPtr& message) {
    if (revision_received_ && message->data != scene_revision_) {
      ClearPlanningMarkers();
    }
    scene_revision_ = message->data;
    revision_received_ = true;
  }

  bool Command(std::uint8_t action,
               buffered_footprint_planner::SetObjectPose::Response* response) {
    buffered_footprint_planner::SetObjectPose service;
    service.request.action = action;
    if (!command_client_.call(service)) return false;
    *response = service.response;
    return service.response.success;
  }

  bool RefreshObstacleSnapshot() {
    for (const auto& obstacle : scenario_.obstacles()) {
      buffered_footprint_planner::SetObjectPose service;
      service.request.action = service.request.QUERY_POSE;
      service.request.object_id = obstacle.id;
      if (!command_client_.call(service) || !service.response.success ||
          !scenario_.SetObstaclePose(obstacle.id, service.response.x,
                                     service.response.y,
                                     service.response.yaw)) {
        return false;
      }
    }
    return true;
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& message) {
    if (planning_) {
      ROS_WARN("A planning task is already running");
      return;
    }
    if (!start_locked_) {
      ROS_WARN("Goal ignored: Vehicle 1 start is not locked");
      return;
    }
    if (!message->header.frame_id.empty() &&
        message->header.frame_id != scenario_.frame_id()) {
      ROS_WARN("Goal ignored: expected frame '%s', got '%s'",
               scenario_.frame_id().c_str(), message->header.frame_id.c_str());
      return;
    }

    buffered_footprint_planner::SetObjectPose::Response command_response;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::BEGIN_PLANNING,
                 &command_response)) {
      ROS_WARN("Cannot start planning: %s", command_response.message.c_str());
      return;
    }
    planning_ = true;
    const Pose2d goal = FromPose(message->pose);
    if (!RefreshObstacleSnapshot() || !ValidVehiclePose(goal)) {
      ROS_WARN("Goal rejected: vehicle leaves map or collides with an obstacle");
      FinishPlanningLock();
      return;
    }

    PlanningSnapshot snapshot;
    snapshot.start = locked_start_;
    snapshot.goal = goal;
    snapshot.bounds = scenario_.bounds();
    snapshot.obstacles = scenario_.obstacle_polygons();
    snapshot.vehicle = vehicle_;
    snapshot.planner = planner_;
    snapshot.planner.xy_bounds = snapshot.bounds;

    QueueGoal(goal);
    VisualizationPlot::ClearMarkers("hybrid_astar_path", 0, 0);
    VisualizationPlot::Trigger();

    planning::HybridAStar planner(snapshot.planner, snapshot.vehicle,
                                  snapshot.obstacles);
    planning::HybridAStartResult result;
    const ros::WallTime begin = ros::WallTime::now();
    const bool success = planner.Plan(
        snapshot.start.x, snapshot.start.y, snapshot.start.yaw,
        snapshot.goal.x, snapshot.goal.y, snapshot.goal.yaw, &result);
    const double elapsed = (ros::WallTime::now() - begin).toSec();

    if (success) {
      std::vector<Color> colors(result.x.size(), Color::Green);
      bool has_reverse = false;
      std::size_t gear_switches = 0;
      for (std::size_t i = 0; i < result.gear.size(); ++i) {
        if (!result.gear[i]) {
          colors[i] = Color::Cyan;
          has_reverse = true;
        }
        if (i > 0 && result.gear[i] != result.gear[i - 1]) ++gear_switches;
      }
      VisualizationPlot::Plot(result.x, result.y, 0.025, colors, 0,
                              "hybrid_astar_path");
      VisualizationPlot::Trigger();
      const double length = result.accumulated_s.empty()
                                ? 0.0
                                : result.accumulated_s.back();
      ROS_INFO("Hybrid A* succeeded: %zu points, %.3f m, reverse=%s, "
               "gear switches=%zu, %.3f s",
               result.x.size(), length, has_reverse ? "yes" : "no",
               gear_switches, elapsed);
    } else {
      ROS_WARN("Hybrid A* failed after %.3f s; scene remains unchanged", elapsed);
    }
    FinishPlanningLock();
  }

  void QueueGoal(const Pose2d& goal) const {
    const auto box = vehicle_.GenerateBox(
        common::math::Pose(goal.x, goal.y, goal.yaw));
    const common::math::Polygon2d footprint(box);
    VisualizationPlot::PlotPolygon(footprint, 0.015, Color::Magenta, 0,
                                   "planning_goal_body");
    const double arrow_length =
        vehicle_.wheel_base + vehicle_.front_hang + 0.04;
    VisualizationPlot::PlotArrow(
        goal.x, goal.y, goal.x + arrow_length * std::cos(goal.yaw),
        goal.y + arrow_length * std::sin(goal.yaw), Color::Magenta, 0,
        "planning_goal_heading");
    VisualizationPlot::PlotPoints({goal.x}, {goal.y}, Color::Magenta,
                                  0.03, 0, "planning_goal_point");
    VisualizationPlot::PlotText(box.max_x() + 0.07, box.max_y() + 0.07,
                                "GOAL", 0.08, Color::Magenta, 0,
                                "planning_goal_label");
  }

  void ClearPlanningMarkers() const {
    VisualizationPlot::ClearMarkers("planning_goal_body", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_heading", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_point", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_label", 0, 0);
    VisualizationPlot::ClearMarkers("hybrid_astar_path", 0, 0);
    VisualizationPlot::Trigger();
  }

  void FinishPlanningLock() {
    buffered_footprint_planner::SetObjectPose::Response response;
    if (!Command(buffered_footprint_planner::SetObjectPoseRequest::END_PLANNING,
                 &response)) {
      ROS_ERROR("Failed to release planning scene: %s", response.message.c_str());
    }
    planning_ = false;
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  VehicleParameter vehicle_;
  ParkingPlannerConfig planner_;
  std::string vehicle_id_ = "vehicle_1";
  Pose2d locked_start_;
  bool start_locked_ = false;
  bool planning_ = false;
  bool revision_received_ = false;
  std::uint64_t scene_revision_ = 0;
  ros::ServiceClient command_client_;
  ros::Subscriber obstacle_subscriber_;
  ros::Subscriber start_subscriber_;
  ros::Subscriber lock_subscriber_;
  ros::Subscriber revision_subscriber_;
  ros::Subscriber goal_subscriber_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_planning");
  PlanningNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
