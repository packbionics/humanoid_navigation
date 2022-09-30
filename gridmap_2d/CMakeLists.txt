################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(gridmap_2d)

################################################################################
# Find packages and libraries for system dependencies
################################################################################
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(OpenCV REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/GridMap2D.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "nav_msgs"
)

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()