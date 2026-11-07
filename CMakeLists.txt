cmake_minimum_required(VERSION 3.5)
project(buffered_footprint_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CASADI REQUIRED casadi)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_service_files(FILES SetObjectPose.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common_math common_util hybrid_astar environment visualization_plot
            footprint_optimizer
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp std_msgs tf2
                 tf2_geometry_msgs visualization_msgs
)

include_directories(
  include
  include/common/math
  ${catkin_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
)
link_directories(${CASADI_LIBRARY_DIRS})

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

add_library(footprint_optimizer
  optimizer/embodied_footprint.cpp
  optimizer/initial_guess.cpp
  optimizer/footprint_nlp.cpp
)
target_link_libraries(footprint_optimizer common_math hybrid_astar
                      ${CASADI_LIBRARIES} ${catkin_LIBRARIES})

add_executable(simulation_node src/simulation_node.cpp)
target_link_libraries(simulation_node environment visualization_plot ${catkin_LIBRARIES})
add_dependencies(simulation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node environment hybrid_astar footprint_optimizer
                      visualization_plot ${catkin_LIBRARIES})
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(keyboard_scene_editor simulation/keyboard_scene_editor.cpp)
target_link_libraries(keyboard_scene_editor ${catkin_LIBRARIES})
add_dependencies(keyboard_scene_editor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS common_math common_util hybrid_astar environment visualization_plot footprint_optimizer
                simulation_node planning_node keyboard_scene_editor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
