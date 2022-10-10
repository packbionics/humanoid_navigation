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

#ifndef FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_
#define FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_

#include <footstep_planner/FootstepPlanner.h>
#include <footstep_planner/State.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <humanoid_nav_msgs/srv/clip_footstep.hpp>
#include <humanoid_nav_msgs/action/exec_footsteps.hpp>
#include <humanoid_nav_msgs/srv/plan_footsteps.hpp>
#include <humanoid_nav_msgs/srv/step_target_service.hpp>

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <bullet/LinearMath/btTransform.h>
#include <bullet/LinearMath/btVector3.h>

#include <rclcpp_action/rclcpp_action.hpp>

#include <assert.h>


namespace footstep_planner
{
/**
 * @brief A class to control the performance of a planned footstep path on
 * the NAO robot.
 */
class FootstepNavigation : public rclcpp::Node
{
public:
  FootstepNavigation();
  virtual ~FootstepNavigation();

  /// @brief Wrapper for FootstepPlanner::setGoal.
  bool setGoal(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_pose);

  /// @brief Wrapper for FootstepPlanner::setGoal.
  bool setGoal(float x, float y, float theta);

  /**
   * @brief Callback to set a simulated robot at a certain pose.
   *
   * Subscribed to 'goal'.
   */
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose);

  /**
   * @brief Callback to set the map.
   *
   * Subscribed to 'map'.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map);

protected:
  /**
   * @brief Starts the planning task from scratch discarding previous planning
   * information.
   *
   * @return Success of the planning.
   */
  bool plan();

  /**
   * @brief Starts the planning task. First FootstepPlanner::replan() is
   * called to use planning information from previous tasks. If this fails
   * FootstepPlanner::plan() is called to plan from scratch. Otherwise
   * the planning task is unsuccessful.
   *
   * @return Success of the planning.
   */
  bool replan();

  /// @brief Starts the execution of the calculated path.
  void startExecution();

  /**
   * @brief Obtains the pose of the robot's foot from tf.
   *
   * @return True if transformation has been received.
   */
  bool getFootTransform(const std::string& foot_id,
                        const std::string& world_frame_id,
                        const rclcpp::Time& time,
                        const rclcpp::Duration& waiting_time,
                        tf2::Transform* foot);

  /**
   * @brief Calculates the footstep necessary to reach 'to' from within
   * 'from'.
   *
   * @return True if the footstep can be performed by the NAO robot.
   */
  bool getFootstep(const tf2::Transform& from, const State& from_planned,
		               const State& to, humanoid_nav_msgs::msg::StepTarget* footstep);

  /**
   * @brief Extracts the footsteps necessary to perform the calculated
   * path.
   *
   * @param current_support_leg The current support leg of the robot. Used
   * to calculate the footstep necessary to reach the calculated path.
   * @param starting_step_num Index of the state in the path which has
   * to be reached first. Usually this is 1 (since state 0 is the current
   * support leg of the robot) but for readjustments indices different
   * from 1 are necessary.
   *
   * @return False if an extracted footstep is invalid.
   */
  bool getFootstepsFromPath(
      const State& current_support_leg, int starting_step_num,
      std::vector<humanoid_nav_msgs::msg::StepTarget>& footsteps);

  /// @brief Updates the robot's current pose.
  bool updateStart();

  /// @brief Executes footsteps as boost::thread.
  void executeFootsteps();

  /**
  * @brief Alternative (and more fluid) execution of footsteps using
  * ROS' actionlib.
  */
  void executeFootstepsFast();

  /**
   * @brief Called from within ROS' actionlib at the start of a new goal
   * request.
   */
  void activeCallback();

  /**
   * @brief Called from within ROS' actionlib at the end of a goal
   * request.
   */
  void doneCallback(const rclcpp_action::ClientGoalHandle<humanoid_nav_msgs::action::ExecFootsteps>::WrappedResult& result);

  /**
   * @brief Called from within ROS' actionlib during the execution of
   * a goal request.
   */
  void feedbackCallback(rclcpp_action::ClientGoalHandle<humanoid_nav_msgs::action::ExecFootsteps>::SharedPtr, const std::shared_ptr<const humanoid_nav_msgs::action::ExecFootsteps::Feedback>& fb);

  bool performable(const humanoid_nav_msgs::msg::StepTarget& footstep);
  bool performable(float step_x, float step_y);

  /**
   * @param footstep The response from the clip footstep service (i.e. it
   * contains the unclipped (calculated) step and the clipped
   * (performable) step).
   *
   * @return True if the footstep can be performed by the robot (i.e. it
   * is within the robot's max ranges).
   */
  // bool performanceValid(const humanoid_nav_msgs::srv::ClipFootstep& footstep);

  /// @return True if both states are equal upon some accuracy.
  bool performanceValid(const State& planned, const State& executed);

  /// @return True if both states are equal upon some accuracy.
  bool performanceValid(float a_x, float a_y, float a_theta,
                        float b_x, float b_y, float b_theta);

  FootstepPlanner ivPlanner;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr ivGridMapSub;
  // rclcpp::Subscription ivRobotPoseSub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ivGoalPoseSub;

  rclcpp::Client<humanoid_nav_msgs::srv::StepTargetService>::SharedPtr ivFootstepSrv;
  rclcpp::Client<humanoid_nav_msgs::srv::ClipFootstep>::SharedPtr ivClipFootstepSrv;

  std::shared_ptr<tf2_ros::Buffer> ivTransformBuffer;
  std::shared_ptr<tf2_ros::TransformListener> ivTransformListener;

  boost::mutex ivExecutionLock;

  boost::shared_ptr<boost::thread> ivFootstepExecutionPtr;

  std::string ivIdFootRight;
  std::string ivIdFootLeft;
  std::string ivIdMapFrame;

  double ivAccuracyX;
  double ivAccuracyY;
  double ivAccuracyTheta;
  double ivCellSize;
  int    ivNumAngleBins;

  /// Search direction.
  bool ivForwardSearch;

  /// Used to lock the calculation and execution of footsteps.
  bool ivExecutingFootsteps;

  /// The rate the action server sends its feedback.
  double ivFeedbackFrequency;

  /// Simple action client to control a footstep execution.
  rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SharedPtr ivFootstepsExecution;

  /// Fixed delay (=2) of the incoming footsteps.
  const int ivExecutionShift;

  /**
   * Index to keep track of the currently executed footstep and the currently
   * observed one.
   */
  int ivControlStepIdx;

  /**
   * Index used to keep track of the currently observed footstep after
   * replanning.
   */
  int ivResetStepIdx;

  /// Whether to use the slower but more cautious execution or not.
  bool ivSafeExecution;

  double ivMaxStepX;
  double ivMaxStepY;
  double ivMaxStepTheta;
  double ivMaxInvStepX;
  double ivMaxInvStepY;
  double ivMaxInvStepTheta;

  std::vector<std::pair<double, double> > ivStepRange;
};
}
#endif  // FOOTSTEP_PLANNER_FOOTSTEPNAVIGATION_H_
