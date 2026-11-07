#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "buffered_footprint_planner/SetObjectPose.h"
#include "common/math/math_utils.h"
#include "environment/scenario_manager.h"
#include "hybrid_astar/vehicle_parameter.h"
#include "visualize/visualization_plot.h"

namespace {

geometry_msgs::Quaternion QuaternionFromYaw(double yaw) {
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

class SimulationNode {
 public:
  SimulationNode() : private_node_("~") {}

  bool Initialize() {
    if (!scenario_.Load(private_node_) || !LoadVehicle()) return false;
    private_node_.param("vehicle/id", vehicle_id_, std::string("vehicle_1"));
    if (!private_node_.getParam("vehicle/x", vehicle_pose_.x) ||
        !private_node_.getParam("vehicle/y", vehicle_pose_.y) ||
        !private_node_.getParam("vehicle/yaw", vehicle_pose_.yaw) ||
        !ValidateVehiclePose(vehicle_pose_)) {
      ROS_ERROR("Invalid initial vehicle pose");
      return false;
    }
    private_node_.param("start_lock/stationary_duration", stationary_duration_, 3.0);
    private_node_.param("start_lock/position_threshold", position_threshold_, 0.002);
    private_node_.param("start_lock/yaw_threshold", yaw_threshold_, 0.0087266463);

    VisualizationPlot::Init(private_node_, scenario_.frame_id(), marker_topic_);
    obstacle_poses_publisher_ = private_node_.advertise<geometry_msgs::PoseArray>(
        "/simulation/scene_obstacle_poses", 1, true);
    vehicle_pose_publisher_ = private_node_.advertise<geometry_msgs::PoseStamped>(
        "/simulation/vehicle_pose", 1, true);
    locked_start_publisher_ = private_node_.advertise<geometry_msgs::PoseStamped>(
        "/simulation/locked_start", 1, true);
    start_locked_publisher_ = private_node_.advertise<std_msgs::Bool>(
        "/simulation/start_locked", 1, true);
    revision_publisher_ = private_node_.advertise<std_msgs::UInt64>(
        "/simulation/scene_revision", 1, true);
    command_service_ = private_node_.advertiseService(
        "/simulation/set_object_pose", &SimulationNode::HandleCommand, this);
    stationary_timer_ = private_node_.createTimer(
        ros::Duration(0.1), &SimulationNode::CheckStationary, this);

    QueueStaticScene();
    QueueVehicle();
    VisualizationPlot::Trigger();
    PublishSceneState();
    ROS_INFO("Scene editor ready; place objects and press Enter in the keyboard editor");
    return true;
  }

 private:
  struct VehiclePose {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  bool LoadVehicle() {
    if (!private_node_.getParam("wheel_base", vehicle_.wheel_base) ||
        !private_node_.getParam("front_hang", vehicle_.front_hang) ||
        !private_node_.getParam("rear_hang", vehicle_.rear_hang) ||
        !private_node_.getParam("width", vehicle_.width)) {
      ROS_ERROR("Missing vehicle geometry in planner.yaml");
      return false;
    }
    vehicle_.GenerateDisc();
    return vehicle_.wheel_base > 0.0 && vehicle_.front_hang >= 0.0 &&
           vehicle_.rear_hang >= 0.0 && vehicle_.width > 0.0;
  }

  common::math::Box2d VehicleBox(const VehiclePose& pose) const {
    return vehicle_.GenerateBox(common::math::Pose(pose.x, pose.y, pose.yaw));
  }

  bool ValidateVehiclePose(const VehiclePose& pose) const {
    const auto box = VehicleBox(pose);
    const auto& bounds = scenario_.bounds();
    for (const auto& corner : box.corners()) {
      if (corner.x() < bounds[0] || corner.x() > bounds[1] ||
          corner.y() < bounds[2] || corner.y() > bounds[3]) return false;
    }
    const common::math::Polygon2d footprint(box);
    for (const auto& obstacle : scenario_.obstacles()) {
      if (footprint.HasOverlap(obstacle.polygon())) return false;
    }
    return true;
  }

  void QueueStaticScene() const {
    const common::math::Polygon2d ground(scenario_.boundary());
    VisualizationPlot::PlotFilledPolygon(ground, Color::White, 0,
                                         "scenario_ground", 0.0);
    VisualizationPlot::PlotPolygon(ground, 0.015, Color::Black, 0,
                                   "scenario_boundary");
    for (std::size_t i = 0; i < scenario_.obstacles().size(); ++i) {
      QueueObstacle(i);
    }
  }

  void QueueObstacle(std::size_t index) const {
    const auto& obstacle = scenario_.obstacles()[index];
    const auto polygon = obstacle.polygon();
    const int marker_id = static_cast<int>(index);
    VisualizationPlot::PlotFilledPolygon(polygon, Color::Grey, marker_id,
                                         "scenario_obstacle_fill", 0.01);
    VisualizationPlot::PlotPolygon(polygon, 0.012, Color::Black, marker_id,
                                   "scenario_obstacle_outline");
    VisualizationPlot::PlotText(obstacle.x, obstacle.y, obstacle.id, 0.12,
                                Color::Black, marker_id,
                                "scenario_obstacle_label");
  }

  void QueueVehicle() const {
    const auto box = VehicleBox(vehicle_pose_);
    const common::math::Polygon2d footprint(box);
    VisualizationPlot::PlotPolygon(footprint, 0.015, Color::Blue, 0,
                                   "simulation_vehicle_body");
    const double nose_distance = vehicle_.wheel_base + vehicle_.front_hang;
    VisualizationPlot::Plot(
        {vehicle_pose_.x,
         vehicle_pose_.x + nose_distance * std::cos(vehicle_pose_.yaw)},
        {vehicle_pose_.y,
         vehicle_pose_.y + nose_distance * std::sin(vehicle_pose_.yaw)},
        0.018, Color::Blue, 0, "simulation_vehicle_heading");
    VisualizationPlot::PlotPoints({vehicle_pose_.x}, {vehicle_pose_.y},
                                  Color::Blue, 0.045, 0,
                                  "simulation_vehicle_rear_axle");
    VisualizationPlot::PlotText(box.max_x() + 0.07, box.max_y() + 0.07,
                                "vehicle1", 0.08, Color::Blue, 0,
                                "simulation_vehicle_label");
  }

  void QueueLockedStart() const {
    const auto box = VehicleBox(locked_start_);
    const common::math::Polygon2d footprint(box);
    VisualizationPlot::PlotPolygon(footprint, 0.012, Color::Green, 0,
                                   "simulation_locked_start");
    VisualizationPlot::PlotPoints({locked_start_.x}, {locked_start_.y},
                                  Color::Green, 0.03, 0,
                                  "simulation_locked_start_rear_axle");
    const double arrow_length =
        vehicle_.wheel_base + vehicle_.front_hang + 0.04;
    VisualizationPlot::PlotArrow(
        locked_start_.x, locked_start_.y,
        locked_start_.x + arrow_length * std::cos(locked_start_.yaw),
        locked_start_.y + arrow_length * std::sin(locked_start_.yaw),
        Color::Green, 0, "simulation_locked_start_heading");
    const double label_x = (box.min_x() + box.max_x()) / 2.0;
    const double label_y = box.min_y() - 0.10;
    VisualizationPlot::PlotText(label_x, label_y, "START LOCKED", 0.08,
                                Color::Green, 0,
                                "simulation_locked_start_label");
  }

  void ClearTaskMarkers() const {
    VisualizationPlot::ClearMarkers("simulation_locked_start", 0, 0);
    VisualizationPlot::ClearMarkers("simulation_locked_start_rear_axle", 0, 0);
    VisualizationPlot::ClearMarkers("simulation_locked_start_heading", 0, 0);
    VisualizationPlot::ClearMarkers("simulation_locked_start_label", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_body", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_heading", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_point", 0, 0);
    VisualizationPlot::ClearMarkers("planning_goal_label", 0, 0);
    VisualizationPlot::ClearMarkers("hybrid_astar_path", 0, 0);
    VisualizationPlot::ClearMarkers("optimized_trajectory", 0, 1023);
  }

  void BeginEditing() {
    if (editing_active_) return;
    editing_active_ = true;
    start_locked_ = false;
    ClearTaskMarkers();
    VisualizationPlot::Trigger();
    PublishSceneState();
    ROS_INFO("Scene editing resumed; previous planning task invalidated");
  }

  bool HandleCommand(
      buffered_footprint_planner::SetObjectPose::Request& request,
      buffered_footprint_planner::SetObjectPose::Response& response) {
    if (request.action == request.BEGIN_PLANNING) {
      if (planning_active_ || editing_active_ || !start_locked_) {
        return FillResponse(false, "Start is not locked or planner is busy",
                            request.object_id, response);
      }
      planning_active_ = true;
      return FillResponse(true, "Planning scene locked", request.object_id,
                          response);
    }
    if (request.action == request.END_PLANNING) {
      planning_active_ = false;
      return FillResponse(true, "Planning scene released", request.object_id,
                          response);
    }
    if (request.action == request.FINISH_EDITING) {
      if (planning_active_) {
        return FillResponse(false, "Planning is active", request.object_id,
                            response);
      }
      editing_active_ = false;
      start_locked_ = false;
      stable_pose_ = vehicle_pose_;
      stable_since_ = ros::Time::now();
      ClearTaskMarkers();
      VisualizationPlot::Trigger();
      PublishSceneState();
      ROS_INFO("Editing finished; checking Vehicle 1 stability for %.1f s",
               stationary_duration_);
      return FillResponse(true, "Stationary detection started", vehicle_id_,
                          response);
    }
    if (request.action == request.QUERY_POSE) {
      return FillResponse(true, "Current pose", request.object_id, response);
    }
    if (request.action != request.SET_POSE) {
      return FillResponse(false, "Unknown command", request.object_id,
                          response);
    }
    if (planning_active_) {
      return FillResponse(false, "Editing is disabled during planning",
                          request.object_id, response);
    }

    BeginEditing();
    const double yaw = common::math::NormalizeAngle(request.yaw);
    if (request.object_id == vehicle_id_) {
      const VehiclePose candidate{request.x, request.y, yaw};
      if (!ValidateVehiclePose(candidate)) {
        return FillResponse(false, "Vehicle leaves map or collides",
                            request.object_id, response);
      }
      vehicle_pose_ = candidate;
      QueueVehicle();
    } else {
      std::size_t index = 0;
      for (; index < scenario_.obstacles().size(); ++index) {
        if (scenario_.obstacles()[index].id == request.object_id) break;
      }
      if (index == scenario_.obstacles().size()) {
        return FillResponse(false, "Unknown object", request.object_id,
                            response);
      }
      if (!scenario_.SetObstaclePose(request.object_id, request.x, request.y,
                                     yaw)) {
        return FillResponse(false, "Obstacle would leave map bounds",
                            request.object_id, response);
      }
      QueueObstacle(index);
    }

    ++scene_revision_;
    VisualizationPlot::Trigger();
    PublishSceneState();
    return FillResponse(true, "Pose updated", request.object_id, response);
  }

  bool FillResponse(
      bool success, const std::string& message, const std::string& object_id,
      buffered_footprint_planner::SetObjectPose::Response& response) const {
    response.success = success;
    response.message = message;
    response.start_locked = start_locked_;
    response.scene_revision = scene_revision_;
    if (object_id == vehicle_id_ || object_id.empty()) {
      response.x = vehicle_pose_.x;
      response.y = vehicle_pose_.y;
      response.yaw = vehicle_pose_.yaw;
      return true;
    }
    for (const auto& obstacle : scenario_.obstacles()) {
      if (obstacle.id == object_id) {
        response.x = obstacle.x;
        response.y = obstacle.y;
        response.yaw = obstacle.yaw;
        return true;
      }
    }
    response.success = false;
    response.message = "Unknown object";
    return true;
  }

  void CheckStationary(const ros::TimerEvent&) {
    if (editing_active_ || planning_active_ || start_locked_) return;
    const double position_change =
        std::hypot(vehicle_pose_.x - stable_pose_.x,
                   vehicle_pose_.y - stable_pose_.y);
    const double yaw_change = std::abs(common::math::NormalizeAngle(
        vehicle_pose_.yaw - stable_pose_.yaw));
    if (position_change > position_threshold_ || yaw_change > yaw_threshold_) {
      stable_pose_ = vehicle_pose_;
      stable_since_ = ros::Time::now();
      return;
    }
    if ((ros::Time::now() - stable_since_).toSec() < stationary_duration_) return;

    locked_start_ = vehicle_pose_;
    start_locked_ = true;
    QueueLockedStart();
    VisualizationPlot::Trigger();
    PublishSceneState();
    ROS_INFO("Vehicle 1 start locked at (%.3f, %.3f, %.1f deg)",
             locked_start_.x, locked_start_.y,
             locked_start_.yaw * 180.0 / M_PI);
  }

  geometry_msgs::PoseStamped ToPoseStamped(const VehiclePose& pose) const {
    geometry_msgs::PoseStamped message;
    message.header.frame_id = scenario_.frame_id();
    message.header.stamp = ros::Time::now();
    message.pose.position.x = pose.x;
    message.pose.position.y = pose.y;
    message.pose.orientation = QuaternionFromYaw(pose.yaw);
    return message;
  }

  void PublishSceneState() const {
    geometry_msgs::PoseArray obstacles;
    obstacles.header.frame_id = scenario_.frame_id();
    obstacles.header.stamp = ros::Time::now();
    for (const auto& obstacle : scenario_.obstacles()) {
      geometry_msgs::Pose pose;
      pose.position.x = obstacle.x;
      pose.position.y = obstacle.y;
      pose.orientation = QuaternionFromYaw(obstacle.yaw);
      obstacles.poses.push_back(pose);
    }
    obstacle_poses_publisher_.publish(obstacles);
    vehicle_pose_publisher_.publish(ToPoseStamped(vehicle_pose_));
    if (start_locked_) locked_start_publisher_.publish(ToPoseStamped(locked_start_));
    std_msgs::Bool locked;
    locked.data = start_locked_;
    start_locked_publisher_.publish(locked);
    std_msgs::UInt64 revision;
    revision.data = scene_revision_;
    revision_publisher_.publish(revision);
  }

  ros::NodeHandle private_node_;
  ScenarioManager scenario_;
  VehicleParameter vehicle_;
  std::string vehicle_id_;
  VehiclePose vehicle_pose_;
  VehiclePose stable_pose_;
  VehiclePose locked_start_;
  bool editing_active_ = true;
  bool planning_active_ = false;
  bool start_locked_ = false;
  double stationary_duration_ = 3.0;
  double position_threshold_ = 0.002;
  double yaw_threshold_ = 0.0087266463;
  ros::Time stable_since_;
  std::uint64_t scene_revision_ = 0;
  const std::string marker_topic_ = "/buffered_footprint_planner/markers";
  ros::Publisher obstacle_poses_publisher_;
  ros::Publisher vehicle_pose_publisher_;
  ros::Publisher locked_start_publisher_;
  ros::Publisher start_locked_publisher_;
  ros::Publisher revision_publisher_;
  ros::ServiceServer command_service_;
  ros::Timer stationary_timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_simulation");
  SimulationNode node;
  if (!node.Initialize()) return EXIT_FAILURE;
  ros::spin();
  return EXIT_SUCCESS;
}
