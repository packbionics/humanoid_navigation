/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.rclcpp.org/wiki/footstep_planner
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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_
#define FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_


#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <footstep_planner/FootstepPlanner.h>


namespace footstep_planner
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class FootstepPlannerNode : public rclcpp::Node
{
public:
  FootstepPlannerNode();
  virtual ~FootstepPlannerNode();

protected:
  FootstepPlanner ivFootstepPlanner;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ivGoalPoseSub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ivGridMapSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ivStartPoseSub;
  // rclcpp::Subscription ivRobotPoseSub;

  rclcpp::Service<humanoid_nav_msgs::srv::PlanFootsteps>::SharedPtr ivFootstepPlanService;
  rclcpp::Service<humanoid_nav_msgs::srv::PlanFootstepsBetweenFeet>::SharedPtr ivFootstepPlanFeetService;
};
}
#endif  // FOOTSTEP_PLANNER_FOOTSTEPPLANNERNODE_H_
