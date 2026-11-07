#include <cstdlib>
#include <vector>

#include <ros/ros.h>

#include "environment/scenario_manager.h"
#include "visualize/visualization_plot.h"

namespace {

void QueueScene(const ScenarioManager& scenario) {
  const auto& boundary = scenario.boundary();
  const common::math::Polygon2d ground(boundary);
  VisualizationPlot::PlotFilledPolygon(ground, Color::White, 0,
                                       "scenario_ground");
  VisualizationPlot::PlotPolygon(ground, 0.015, Color::Black, 0,
                                 "scenario_boundary");

  const auto& obstacles = scenario.obstacles();
  for (std::size_t i = 0; i < obstacles.size(); ++i) {
    const auto polygon = obstacles[i].polygon();
    const int marker_id = static_cast<int>(i);
    VisualizationPlot::PlotFilledPolygon(polygon, Color::Grey, marker_id,
                                         "scenario_obstacle_fill");
    VisualizationPlot::PlotPolygon(polygon, 0.012, Color::Black, marker_id,
                                   "scenario_obstacle_outline");
    VisualizationPlot::PlotText(obstacles[i].x, obstacles[i].y,
                                obstacles[i].id, 0.12, Color::Black,
                                marker_id, "scenario_obstacle_label");
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "buffered_footprint_simulation");
  ros::NodeHandle private_node("~");

  ScenarioManager scenario;
  if (!scenario.Load(private_node)) return EXIT_FAILURE;

  VisualizationPlot::Init(private_node, scenario.frame_id(),
                          "/buffered_footprint_planner/markers");
  QueueScene(scenario);
  VisualizationPlot::Trigger();

  ROS_INFO("Loaded %.1f x %.1f m scene with %zu obstacles in frame '%s'",
           scenario.bounds()[1] - scenario.bounds()[0],
           scenario.bounds()[3] - scenario.bounds()[2],
           scenario.obstacles().size(), scenario.frame_id().c_str());
  ros::spin();
  return EXIT_SUCCESS;
}
