cmake_minimum_required(VERSION 3.5)
project(buffered_footprint_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common_math common_util hybrid_astar environment visualization_plot
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs visualization_msgs
)

include_directories(
  include
  include/common/math
  ${catkin_INCLUDE_DIRS}
)

add_library(common_math
  common/math/aabox2d.cc
  common/math/box2d.cc
  common/math/line_segment2d.cc
  common/math/math_utils.cc
  common/math/polygon2d.cc
  common/math/vec2d.cc
)

add_library(common_util
  common/util/color.cc
  common/util/csv_logger.cc
)
target_link_libraries(common_util common_math ${catkin_LIBRARIES})

add_library(hybrid_astar
  hybrid_astar/grid_search.cc
  hybrid_astar/hybrid_a_star.cc
  hybrid_astar/node3d.cc
  hybrid_astar/reeds_shepp_path.cc
)
target_link_libraries(hybrid_astar common_math common_util)

add_library(environment environment/scenario_manager.cpp)
target_link_libraries(environment common_math ${catkin_LIBRARIES})

add_library(visualization_plot visualize/visualization_plot.cpp)
target_link_libraries(visualization_plot common_math common_util ${catkin_LIBRARIES})

add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node environment visualization_plot ${catkin_LIBRARIES})

install(TARGETS common_math common_util hybrid_astar environment visualization_plot simulation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
