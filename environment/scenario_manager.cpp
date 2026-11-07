#include "environment/scenario_manager.h"

#include <cmath>

#include <XmlRpcValue.h>

#include "common/math/box2d.h"

namespace {

bool ReadNumber(const XmlRpc::XmlRpcValue& value, const std::string& key,
                double* result) {
  if (!value.hasMember(key)) return false;
  const auto& field = value[key];
  if (field.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    *result = static_cast<double>(field);
    return std::isfinite(*result);
  }
  if (field.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    *result = static_cast<int>(field);
    return true;
  }
  return false;
}

}  // namespace

common::math::Polygon2d ScenarioManager::Obstacle::polygon() const {
  return common::math::Polygon2d(
      common::math::Box2d({x, y}, yaw, length, width));
}

bool ScenarioManager::Load(const ros::NodeHandle& node) {
  node.param("frame_id", frame_id_, std::string("map"));
  if (!node.getParam("map/x_min", bounds_[0]) ||
      !node.getParam("map/x_max", bounds_[1]) ||
      !node.getParam("map/y_min", bounds_[2]) ||
      !node.getParam("map/y_max", bounds_[3]) ||
      bounds_[0] >= bounds_[1] || bounds_[2] >= bounds_[3]) {
    ROS_ERROR("Invalid or missing map bounds in env.yaml");
    return false;
  }

  XmlRpc::XmlRpcValue values;
  if (!node.getParam("obstacles", values) ||
      values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("Parameter 'obstacles' must be an array");
    return false;
  }

  obstacles_.clear();
  for (int i = 0; i < values.size(); ++i) {
    const auto& value = values[i];
    Obstacle obstacle;
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
        !value.hasMember("id") ||
        value["id"].getType() != XmlRpc::XmlRpcValue::TypeString ||
        !ReadNumber(value, "length", &obstacle.length) ||
        !ReadNumber(value, "width", &obstacle.width) ||
        !ReadNumber(value, "x", &obstacle.x) ||
        !ReadNumber(value, "y", &obstacle.y) ||
        !ReadNumber(value, "yaw", &obstacle.yaw) ||
        obstacle.length <= 0.0 || obstacle.width <= 0.0) {
      ROS_ERROR("Invalid obstacle entry at index %d", i);
      return false;
    }
    obstacle.id = static_cast<std::string>(value["id"]);
    const auto polygon = obstacle.polygon();
    if (polygon.min_x() < bounds_[0] || polygon.max_x() > bounds_[1] ||
        polygon.min_y() < bounds_[2] || polygon.max_y() > bounds_[3]) {
      ROS_ERROR("Obstacle '%s' lies outside the map", obstacle.id.c_str());
      return false;
    }
    obstacles_.push_back(obstacle);
  }

  boundary_ = {{bounds_[0], bounds_[2]}, {bounds_[1], bounds_[2]},
               {bounds_[1], bounds_[3]}, {bounds_[0], bounds_[3]}};
  return true;
}

bool ScenarioManager::SetObstaclePose(const std::string& id, double x,
                                      double y, double yaw) {
  for (auto& obstacle : obstacles_) {
    if (obstacle.id == id) {
      Obstacle candidate = obstacle;
      candidate.x = x;
      candidate.y = y;
      candidate.yaw = yaw;
      const auto polygon = candidate.polygon();
      if (polygon.min_x() < bounds_[0] || polygon.max_x() > bounds_[1] ||
          polygon.min_y() < bounds_[2] || polygon.max_y() > bounds_[3]) {
        return false;
      }
      obstacle.x = x;
      obstacle.y = y;
      obstacle.yaw = yaw;
      return true;
    }
  }
  return false;
}

std::vector<common::math::Polygon2d> ScenarioManager::obstacle_polygons() const {
  std::vector<common::math::Polygon2d> polygons;
  polygons.reserve(obstacles_.size());
  for (const auto& obstacle : obstacles_) polygons.push_back(obstacle.polygon());
  return polygons;
}
