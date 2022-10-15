################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(footstep_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find packages and libraries for system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(humanoid_nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(angles REQUIRED)
find_package(Boost REQUIRED)

find_package(PkgConfig REQUIRED)
find_package(sbpl REQUIRED)

find_package(OpenCV REQUIRED)

find_package(gridmap_2d REQUIRED)

include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

################################################################################
# Build
################################################################################
include_directories(
  include
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
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclpy"
  "angles"
  "sensor_msgs"
  "visualization_msgs"
  "humanoid_nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rclcpp_action"
  "OpenCV"
  "gridmap_2d"
)

add_executable(footstep_planner_node src/footstep_planner.cpp)
target_include_directories(footstep_planner_node PUBLIC)
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${SBPL_LIBRARIES})
ament_target_dependencies(footstep_planner_node
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "rclpy"
  "angles"
  "sensor_msgs"
  "visualization_msgs"
  "humanoid_nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rclcpp_action"
  "OpenCV"
  "gridmap_2d"
)


# add_executable(footstep_planner_walls src/footstep_planner_walls.cpp)
# target_link_libraries(footstep_planner_walls ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${SBPL_LIBRARIES})

# add_executable(footstep_navigation_node src/footstep_navigation.cpp)
# target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${SBPL_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS footstep_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()

################################################################################
# Test
################################################################################
