/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <footstep_planner/FootstepPlannerNode.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS 2 initialized!");

  auto node = std::make_shared<footstep_planner::FootstepPlannerNode>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Spinning node...");
  rclcpp::spin(node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ROS 2 shutting down...");
  rclcpp::shutdown();
  return 0;
}
