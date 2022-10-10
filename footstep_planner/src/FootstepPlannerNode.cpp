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

namespace footstep_planner
{
  FootstepPlannerNode::FootstepPlannerNode() : rclcpp::Node("footstep_planner_node")
  {
    RCLCPP_INFO(this->get_logger(), "Node created!");

    // provide callbacks to interact with the footstep planner:
    ivGridMapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&FootstepPlanner::mapCallback, &ivFootstepPlanner, std::placeholders::_1));
    ivGoalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&FootstepPlanner::goalPoseCallback, &ivFootstepPlanner, std::placeholders::_1));
    ivStartPoseSub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&FootstepPlanner::startPoseCallback, &ivFootstepPlanner, std::placeholders::_1));

    // service:
    ivFootstepPlanService = this->create_service<humanoid_nav_msgs::srv::PlanFootsteps>("plan_footsteps", std::bind(&FootstepPlanner::planService, &ivFootstepPlanner, std::placeholders::_1, std::placeholders::_2));
    ivFootstepPlanFeetService = this->create_service<humanoid_nav_msgs::srv::PlanFootstepsBetweenFeet>("plan_footsteps_feet", std::bind(&FootstepPlanner::planFeetService, &ivFootstepPlanner, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Subscriptions and services created!");
  }


  FootstepPlannerNode::~FootstepPlannerNode()
  {}
}
