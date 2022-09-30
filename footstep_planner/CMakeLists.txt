################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(footstep_planner)

set(GRIDMAP_SHARE_DIR "/home/anthony/dev-ws/install/gridmap_2d/share")
set(HUMANOID_NAV_INTERFACES_DIR "/home/anthony/dev-ws/install/humanoid_nav_msgs/include/humanoid_nav_msgs")

set(CMAKE_PREFIX_PATH "/home/anthony/dev-ws/install/")

################################################################################
# Find packages and libraries for system dependencies
################################################################################
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(actionlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(humanoid_nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(angles REQUIRED)
find_package(gridmap_2d REQUIRED)
#find_package(map_server REQUIRED)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

################################################################################
# Build
################################################################################
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)

set(FOOTSTEP_PLANNER_FILES 
  src/FootstepPlanner.cpp
	src/FootstepNavigation.cpp
  src/FootstepPlannerNode.cpp
  src/FootstepPlannerEnvironment.cpp 
  src/Footstep.cpp
  src/PlanningState.cpp
  src/Heuristic.cpp 
  src/helper.cpp
  src/PathCostHeuristic.cpp
  src/PlanningStateChangeQuery.cpp
  src/State.cpp
)

add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
target_link_libraries(${PROJECT_NAME}  ${SBPL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${GRIDMAP_SHARE_DIR} ${HUMANOID_NAV_INTERFACES_DIR})
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclpy"
  "gridmap_2d"
  "angles"
  "sensor_msgs"
  "visualization_msgs"
  "humanoid_nav_msgs"
  "tf2"
  "tf2_ros"
)

#add_executable(footstep_planner_node src/footstep_planner.cpp)
#target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${SBPL_LIBRARIES})

# add_executable(footstep_planner_walls src/footstep_planner_walls.cpp)
# target_link_libraries(footstep_planner_walls ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${SBPL_LIBRARIES})

# add_executable(footstep_navigation_node src/footstep_navigation.cpp)
# target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${SBPL_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#install(TARGETS footstep_planner_node footstep_planner_walls footstep_navigation_node
#  RUNTIME DESTINATION bin
#)

install(PROGRAMS
  scripts/corrected_initialpose.py
  scripts/plan_footsteps.py
  DESTINATION script/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/include/${PROJECT_NAME}
)

install(DIRECTORY config launch maps 
	DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Test
################################################################################
