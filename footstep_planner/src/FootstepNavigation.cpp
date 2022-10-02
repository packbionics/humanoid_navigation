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

#include <footstep_planner/FootstepNavigation.h>


namespace footstep_planner
{
FootstepNavigation::FootstepNavigation()
: rclcpp::Node("footstep_navigation"),
  ivIdFootRight("/r_sole"),
  ivIdFootLeft("/l_sole"),
  ivIdMapFrame("map"),
  ivExecutingFootsteps(false),
  ivExecutionShift(2),
  ivControlStepIdx(-1),
  ivResetStepIdx(0)
{
  ivTransformBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  ivTransformListener = std::make_shared<tf2_ros::TransformListener>(*ivTransformBuffer);

  // action client
  ivFootstepsExecution = rclcpp_action::create_client<humanoid_nav_msgs::action::ExecFootsteps>(this, "footsteps_execution");

  // service
  ivFootstepSrv = this->create_client<humanoid_nav_msgs::srv::StepTargetService>("footstep_srv");
  ivClipFootstepSrv = this->create_client<humanoid_nav_msgs::srv::ClipFootstep>("clip_footstep_srv");

  // subscribers
  ivGridMapSub = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, std::bind(&FootstepNavigation::mapCallback, this, std::placeholders::_1));
  ivGoalPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal", 1, std::bind(&FootstepNavigation::goalPoseCallback, this, std::placeholders::_1));

  // read parameters from config file:
  ivIdFootRight = this->declare_parameter("rfoot_frame_id", ivIdFootRight);
  ivIdFootLeft = this->declare_parameter("lfoot_frame_id", ivIdFootLeft);

  ivAccuracyX = this->declare_parameter("accuracy/footstep/x", 0.01);
  ivAccuracyY = this->declare_parameter("accuracy/footstep/y", 0.01);
  ivAccuracyTheta = this->declare_parameter("accuracy/footstep/theta", 0.1);

  ivCellSize = this->declare_parameter("accuracy/cell_size", 0.005);
  ivNumAngleBins = this->declare_parameter("accuracy/num_angle_bins", 128);

  ivForwardSearch = this->declare_parameter("forward_search", false);

  ivFeedbackFrequency = this->declare_parameter("feedback_frequency", 5.0);
  ivSafeExecution = this->declare_parameter("safe_execution", true);

  ivMaxStepX = this->declare_parameter("foot/max/step/x", 0.07);
  ivMaxStepY = this->declare_parameter("foot/max/step/y", 0.15);
  ivMaxStepTheta = this->declare_parameter("foot/max/step/theta", 0.3);
  ivMaxInvStepX = this->declare_parameter("foot/max/inverse/step/x", -0.03);
  ivMaxInvStepY = this->declare_parameter("foot/max/inverse/step/y", 0.09);
  ivMaxInvStepTheta = this->declare_parameter("foot/max/inverse/step/theta", -0.01);

  // step range
  std::vector<double> step_range_x;
  std::vector<double> step_range_y;
  step_range_x = this->get_parameter("step_range/x").as_double_array();
  step_range_y = this->get_parameter("step_range/y").as_double_array();
  if (step_range_x.size() != step_range_y.size())
  {
    RCLCPP_ERROR(this->get_logger(), "Step range points have different size. Exit!");
    exit(2);
  }
  // create step range
  ivStepRange.clear();
  ivStepRange.reserve(step_range_x.size());
  double x, y;
  for (int i = 0; i < step_range_x.size(); ++i)
  {
    x = (double)step_range_x[i];
    y = (double)step_range_y[i];
    ivStepRange.push_back(std::pair<double, double>(x, y));
  }
  // insert first point again at the end!
  ivStepRange.push_back(ivStepRange[0]);
}


FootstepNavigation::~FootstepNavigation()
{}


bool
FootstepNavigation::plan()
{
  if (!updateStart())
  {
    RCLCPP_ERROR(this->get_logger(), "Start pose not accessible!");
    return false;
  }

  if (ivPlanner.plan())
  {
    startExecution();
    return true;
  }
  // path planning unsuccessful
  return false;
}


bool
FootstepNavigation::replan()
{
  if (!updateStart())
  {
    RCLCPP_ERROR(this->get_logger(), "Start pose not accessible!");
    return false;
  }

  bool path_existed = ivPlanner.pathExists();

  // calculate path by replanning (if no planning information exists
  // this call is equal to ivPlanner.plan())
  if (ivPlanner.replan())
  {
    startExecution();
    return true;
  }
  else if (path_existed)
  {
    RCLCPP_INFO(this->get_logger(), "Replanning unsuccessful. Reseting previous planning "
             "information.");
    if (ivPlanner.plan())
    {
      startExecution();
      return true;
    }
  }
  // path planning unsuccessful
  ivExecutingFootsteps = false;
  return false;
}


void
FootstepNavigation::startExecution()
{
  if (ivSafeExecution)
  {
    ivFootstepExecutionPtr.reset(
      new boost::thread(
        std::bind(&FootstepNavigation::executeFootsteps, this)));
  }
  else
  {
    // ALTERNATIVE:
    executeFootstepsFast();
  }
}


void
FootstepNavigation::executeFootsteps()
{
  if (ivPlanner.getPathSize() <= 1)
    return;

  // lock this thread
  ivExecutingFootsteps = true;

  RCLCPP_INFO(this->get_logger(), "Start walking towards the goal.");

  humanoid_nav_msgs::msg::StepTarget step;
  std::shared_ptr<humanoid_nav_msgs::srv::StepTargetService::Request> step_srv_request;

  tf2::Transform from;
  std::string support_foot_id;

  // calculate and perform relative footsteps until goal is reached
  state_iter_t to_planned = ivPlanner.getPathBegin();
  if (to_planned == ivPlanner.getPathEnd())
  {
    RCLCPP_ERROR(this->get_logger(), "No plan available. Return.");
    return;
  }

  const State* from_planned = to_planned.base();
  to_planned++;
  while (to_planned != ivPlanner.getPathEnd())
  {
    try
    {
      boost::this_thread::interruption_point();
    }
    catch (const boost::thread_interrupted&)
    {
      // leave this thread
      return;
    }

    if (from_planned->getLeg() == RIGHT)
      support_foot_id = ivIdFootRight;
    else // support_foot = LLEG
      support_foot_id = ivIdFootLeft;

    // try to get real placement of the support foot
    if (getFootTransform(support_foot_id, ivIdMapFrame, this->get_clock()->now(),
                         rclcpp::Duration(0, 5 * 100000000), &from))
    {
      // calculate relative step and check if it can be performed
      if (getFootstep(from, *from_planned, *to_planned, &step))
      {
        step_srv_request->step = step;
        ivFootstepSrv->async_send_request(step_srv_request);
      }
      // ..if it cannot be performed initialize replanning
      else
      {
        RCLCPP_INFO(this->get_logger(), "Footstep cannot be performed. Replanning necessary.");

        replan();
        // leave the thread
        return;
      }
    }
    else
    {
      // if the support foot could not be received wait and try again
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    from_planned = to_planned.base();
    to_planned++;
  }
  RCLCPP_INFO(this->get_logger(), "Succeeded walking to the goal.\n");

  // free the lock
  ivExecutingFootsteps = false;
}


void
FootstepNavigation::executeFootstepsFast()
{
  if (ivPlanner.getPathSize() <= 1)
	return;

  // lock the planning and execution process
  ivExecutingFootsteps = true;

  // make sure the action client is connected to the action server
  ivFootstepsExecution->wait_for_action_server();

  humanoid_nav_msgs::action::ExecFootsteps::Goal goal;
  State support_leg;
  if (ivPlanner.getPathBegin()->getLeg() == RIGHT)
    support_leg = ivPlanner.getStartFootRight();
  else // leg == LEFT
    support_leg = ivPlanner.getStartFootLeft();
  if (getFootstepsFromPath(support_leg, 1, goal.footsteps))
  {
    goal.feedback_frequency = ivFeedbackFrequency;
    ivControlStepIdx = 0;
    ivResetStepIdx = 0;

    // start the execution via action; std::placeholders::_1, std::placeholders::_2 are place holders for
    // function arguments (see boost doc)
    rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions send_goal_options = rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions();
      
      send_goal_options.goal_response_callback = std::bind(&FootstepNavigation::activeCallback, this);
      send_goal_options.feedback_callback = std::bind(&FootstepNavigation::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&FootstepNavigation::doneCallback, this, std::placeholders::_1);

      ivFootstepsExecution->async_send_goal(goal, send_goal_options);
  }
  else
  {
    // free the lock
    ivExecutingFootsteps = false;

    replan();
  }
}


void
FootstepNavigation::activeCallback()
{
	// lock the execution
	ivExecutingFootsteps = true;

	RCLCPP_INFO(this->get_logger(), "Start walking towards the goal.");
}


void
FootstepNavigation::doneCallback(const rclcpp_action::ClientGoalHandle<humanoid_nav_msgs::action::ExecFootsteps>::WrappedResult& result)
{
	if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
		RCLCPP_INFO(this->get_logger(), "Succeeded walking to the goal.");
	else if (result.code == rclcpp_action::ResultCode::CANCELED)
		RCLCPP_INFO(this->get_logger(), "Canceled walking to the goal.");
	// TODO: distinct between further states??
	else
		RCLCPP_INFO(this->get_logger(), "Failed walking to the goal.");

	// free the lock
	ivExecutingFootsteps = false;
}


void FootstepNavigation::feedbackCallback(rclcpp_action::ClientGoalHandle<humanoid_nav_msgs::action::ExecFootsteps>::SharedPtr goal_handle, const std::shared_ptr<const humanoid_nav_msgs::action::ExecFootsteps::Feedback>& fb)
{
	int executed_steps_idx = fb->executed_footsteps.size() - ivExecutionShift;
	// make sure at least one step has been performed
	if (executed_steps_idx < 0)
    return;
	// if the currently executed footstep equals the currently observed one
	// everything is ok
	if (executed_steps_idx == ivControlStepIdx)
    return;

	// get planned foot placement
  const State& planned = *(ivPlanner.getPathBegin() + ivControlStepIdx + 1 +
                           ivResetStepIdx);
  // get executed foot placement
  tf2::Transform executed_tf;
  std::string foot_id;
  if (planned.getLeg() == RIGHT)
    foot_id = ivIdFootRight;
  else
    foot_id = ivIdFootLeft;

  if (!getFootTransform(foot_id, ivIdMapFrame, this->get_clock()->now(),
		                    rclcpp::Duration(0, 5 * 100000000), &executed_tf))
  {
    State executed(executed_tf.getOrigin().x(), executed_tf.getOrigin().y(),
                   tf2::getYaw(executed_tf.getRotation()), planned.getLeg());
    ivFootstepsExecution->async_cancel_goal(goal_handle);
    humanoid_nav_msgs::action::ExecFootsteps::Goal goal;
    // try to reach the calculated path
    if (getFootstepsFromPath(executed, executed_steps_idx + ivResetStepIdx,
                             goal.footsteps))
    {
      goal.feedback_frequency = ivFeedbackFrequency;
      // adjust the internal counters
      ivResetStepIdx += ivControlStepIdx + 1;
      ivControlStepIdx = 0;

      rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions send_goal_options = rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions();
      
      send_goal_options.goal_response_callback = std::bind(&FootstepNavigation::activeCallback, this);
      send_goal_options.feedback_callback = std::bind(&FootstepNavigation::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&FootstepNavigation::doneCallback, this, std::placeholders::_1);

      ivFootstepsExecution->async_send_goal(goal, send_goal_options);
    }
    // the previously calculated path cannot be reached so we have plan
    // a new path
    else
    {
      replan();
    }
  }

  State executed(executed_tf.getOrigin().x(), executed_tf.getOrigin().y(),
                 tf2::getYaw(executed_tf.getRotation()), planned.getLeg());

  // check if the currently executed footstep is no longer observed (i.e.
  // the robot no longer follows its calculated path)
  if (executed_steps_idx >= ivControlStepIdx + 2)
	{
    ivFootstepsExecution->async_cancel_goal(goal_handle);

    RCLCPP_DEBUG(this->get_logger(), "Footstep execution incorrect.");

    humanoid_nav_msgs::action::ExecFootsteps::Goal goal;
    // try to reach the calculated path
    if (getFootstepsFromPath(executed, executed_steps_idx + ivResetStepIdx,
                             goal.footsteps))
    {
      RCLCPP_INFO(this->get_logger(), "Try to reach calculated path.");

      goal.feedback_frequency = ivFeedbackFrequency;
      // adjust the internal counters
      ivResetStepIdx += ivControlStepIdx + 1;
      ivControlStepIdx = 0;

      // restart the footstep execution
      rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions send_goal_options = rclcpp_action::Client<humanoid_nav_msgs::action::ExecFootsteps>::SendGoalOptions();
      
      send_goal_options.goal_response_callback = std::bind(&FootstepNavigation::activeCallback, this);
      send_goal_options.feedback_callback = std::bind(&FootstepNavigation::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&FootstepNavigation::doneCallback, this, std::placeholders::_1);

      ivFootstepsExecution->async_send_goal(goal, send_goal_options);
    }
    // the previously calculated path cannot be reached so we have plan
    // a new path
    else
    {
      replan();
    }

    return;
	}
    // check the currently observed footstep
	else
	{
    RCLCPP_DEBUG(this->get_logger(), "planned (%f, %f, %f, %i) vs. executed (%f, %f, %f, %i)",
              planned.getX(), planned.getY(), planned.getTheta(),
              planned.getLeg(),
              executed.getX(), executed.getY(), executed.getTheta(),
              executed.getLeg());

    // adjust the internal step counters if the footstep has been
    // performed correctly; otherwise check in the next iteration if
    // the step really has been incorrect
    if (performanceValid(planned, executed))
      ivControlStepIdx++;
    else
      RCLCPP_DEBUG(this->get_logger(), "Invalid step. Wait next step update before declaring"
                " step incorrect.");
	}
}


void FootstepNavigation::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose)
{
  // check if the execution is locked
  if (ivExecutingFootsteps)
  {
    RCLCPP_INFO(this->get_logger(), "Already performing a navigation task. Wait until it is "
             "finished.");
    return;
  }

  if (setGoal(goal_pose))
  {
    // this check enforces a planning from scratch if necessary (dependent on
    // planning direction)
	  if (ivForwardSearch)
	    replan();
	  else
	    plan();
  }
}


void
FootstepNavigation::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map)
{
  // stop execution if an execution was performed
  if (ivExecutingFootsteps)
  {
    if (ivSafeExecution)
    {
      // interrupt the thread and wait until it has finished its execution
  	  ivFootstepExecutionPtr->interrupt();
      ivFootstepExecutionPtr->join();
    }
    else
    {
      ivFootstepsExecution->async_cancel_all_goals();
    }
  }

  std::shared_ptr<gridmap_2d::GridMap2D> map(new gridmap_2d::GridMap2D(occupancy_map));
  ivIdMapFrame = map->getFrameID();

  // updates the map and starts replanning if necessary
  if (ivPlanner.updateMap(map))
  {
    replan();
  }
}


bool
FootstepNavigation::setGoal(const std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_pose)
{
  return setGoal(goal_pose->pose.position.x,
                 goal_pose->pose.position.y,
                 tf2::getYaw(goal_pose->pose.orientation));
}


bool
FootstepNavigation::setGoal(float x, float y, float theta)
{
	return ivPlanner.setGoal(x, y, theta);
}


bool
FootstepNavigation::updateStart()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  tf2::Transform foot_left, foot_right;
  {
    // get real placement of the feet
	  if (!getFootTransform(ivIdFootLeft, ivIdMapFrame, this->get_clock()->now(),
      		                rclcpp::Duration(0, 5 * 100000000), &foot_left))
	  {
	    if (ivPlanner.pathExists())
	    {
	      ivExecutingFootsteps = false;
	    }
	    return false;
	  }
    if (!getFootTransform(ivIdFootRight, ivIdMapFrame, this->get_clock()->now(),
    		                  rclcpp::Duration(0, 5 * 100000000), &foot_right))
    {
      if (ivPlanner.pathExists())
      {
        ivExecutingFootsteps = false;
      }
      return false;
    }
  }
  State left(foot_left.getOrigin().x(), foot_left.getOrigin().y(),
  		       tf2::getYaw(foot_left.getRotation()), LEFT);
  State right(foot_right.getOrigin().x(), foot_right.getOrigin().y(),
              tf2::getYaw(foot_right.getRotation()), RIGHT);

  RCLCPP_INFO(this->get_logger(), "Robot standing at (%f, %f, %f, %i) (%f, %f, %f, %i).",
		       left.getX(), left.getY(), left.getTheta(), left.getLeg(),
		       right.getX(), right.getY(), right.getTheta(), right.getLeg());

  return ivPlanner.setStart(left, right);
}


bool
FootstepNavigation::getFootstep(const tf2::Transform& from,
                                const State& from_planned,
		                            const State& to,
		                            humanoid_nav_msgs::msg::StepTarget* footstep)
{
  // get footstep to reach 'to' from 'from'
  tf2::Transform step = from.inverse() *
                       tf2::Transform(tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), to.getTheta()),
                                tf2::Vector3(to.getX(), to.getY(), 0.0));

  // set the footstep
  footstep->pose.x = step.getOrigin().x();
  footstep->pose.y = step.getOrigin().y();
  footstep->pose.theta = tf2::getYaw(step.getRotation());
  if (to.getLeg() == LEFT)
    footstep->leg = humanoid_nav_msgs::msg::StepTarget::LEFT;
  else // to.leg == RIGHT
    footstep->leg = humanoid_nav_msgs::msg::StepTarget::RIGHT;


  /* check if the footstep can be performed by the NAO robot ******************/

  // check if the step lies within the executable range
  if (performable(*footstep))
  {
    return true;
  }
  else
  {
    // check if there is only a minor divergence between the current support
	// foot and the foot placement used during the plannig task: in such a case
	// perform the step that has been used during the planning
    float step_diff_x = fabs(from.getOrigin().x() - from_planned.getX());
    float step_diff_y = fabs(from.getOrigin().y() - from_planned.getY());
    float step_diff_theta = fabs(
        angles::shortest_angular_distance(
            tf2::getYaw(from.getRotation()), from_planned.getTheta()));
    if (step_diff_x < ivAccuracyX && step_diff_y < ivAccuracyY &&
        step_diff_theta < ivAccuracyTheta)
    {
	  step = tf2::Transform(tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), from_planned.getTheta()),
	                  tf2::Vector3(from_planned.getX(), from_planned.getY(), 0.0)
	                  ).inverse() *
		     tf2::Transform(tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), to.getTheta()),
				      tf2::Vector3(to.getX(), to.getY(), 0.0));

	  footstep->pose.x = step.getOrigin().x();
	  footstep->pose.y = step.getOrigin().y();
	  footstep->pose.theta = tf2::getYaw(step.getRotation());

	  return true;
    }

    return false;
  }

//  // ALTERNATIVE: clip the footstep into the executable range; if nothing was
//  // clipped: perform; if too much was clipped: do not perform
//  humanoid_nav_msgs::ClipFootstep footstep_clip;
//  footstep_clip.request.step = footstep;
//  ivClipFootstepSrv.call(footstep_clip);
//
//  if (performanceValid(footstep_clip))
//  {
//  	footstep.pose.x = footstep_clip.response.step.pose.x;
//  	footstep.pose.y = footstep_clip.response.step.pose.y;
//  	footstep.pose.theta = footstep_clip.response.step.pose.theta;
//  	return true;
//  }
//  else
//  {
//    return false;
//  }
}


bool
FootstepNavigation::getFootstepsFromPath(
  const State& current_support_leg, int starting_step_num,
  std::vector<humanoid_nav_msgs::msg::StepTarget>& footsteps)
{
  humanoid_nav_msgs::msg::StepTarget footstep;

  state_iter_t to_planned = ivPlanner.getPathBegin() + starting_step_num - 1;
  tf2::Transform last(tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), current_support_leg.getTheta()),
                tf2::Vector3(current_support_leg.getX(), current_support_leg.getY(),
                          0.0));
  const State* from_planned = to_planned.base();
  to_planned++;
  for (; to_planned != ivPlanner.getPathEnd(); to_planned++)
  {
    if (getFootstep(last, *from_planned, *to_planned, &footstep))
    {
      footsteps.push_back(footstep);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Calculated path cannot be performed!");
      return false;
    }

    last = tf2::Transform(tf2::Quaternion(tf2::Vector3(1.0, 0.0, 0.0), to_planned->getTheta()),
                    tf2::Vector3(to_planned->getX(), to_planned->getY(), 0.0));
    from_planned = to_planned.base();
  }

  return true;
}


bool
FootstepNavigation::getFootTransform(const std::string& foot_id,
                                     const std::string& world_frame_id,
                                     const rclcpp::Time& time,
                                     const rclcpp::Duration& waiting_time,
                                     tf2::Transform* foot)
{
  tf2::Stamped<tf2::Transform> stamped_foot_transform;
  try
  { 
    tf2::Transform tmp;
    
    ivTransformBuffer->waitForTransform(foot_id, world_frame_id, time, waiting_time, nullptr);
    tf2::fromMsg(ivTransformBuffer->lookupTransform(foot_id, world_frame_id, time).transform, tmp);

    stamped_foot_transform.setData(tmp);
  }
  catch (const tf2::TransformException& e)
  {
    RCLCPP_WARN(this->get_logger(), "Failed to obtain FootTransform from tf (%s)", e.what());
    return false;
  }

  foot->setOrigin(tf2::Vector3(stamped_foot_transform.getOrigin().getX(), stamped_foot_transform.getOrigin().getY(), stamped_foot_transform.getOrigin().getZ()));
  foot->setRotation(tf2::Quaternion(stamped_foot_transform.getRotation().getX(), stamped_foot_transform.getRotation().getY(), stamped_foot_transform.getRotation().getZ(), stamped_foot_transform.getRotation().getW()));

  return true;
}


bool
FootstepNavigation::performanceValid(float a_x, float a_y, float a_theta,
                                     float b_x, float b_y, float b_theta)
{
  return (fabs(a_x - b_x) < ivAccuracyX &&
          fabs(a_y - b_y) < ivAccuracyY &&
          fabs(angles::shortest_angular_distance(a_theta, b_theta)) <
            ivAccuracyTheta);
}


// bool FootstepNavigation::performanceValid(const humanoid_nav_msgs::srv::ClipFootstep& step)
// {
//   return performanceValid(step.request.step.pose.x,
//                           step.request.step.pose.y,
//                           step.request.step.pose.theta,
//                           step.response.step.pose.x,
//                           step.response.step.pose.y,
//                           step.response.step.pose.theta);
// }


bool
FootstepNavigation::performanceValid(const State& planned,
                                     const State& executed)
{
  return performanceValid(
    planned.getX(), planned.getY(), planned.getTheta(),
    executed.getX(), executed.getY(), executed.getTheta());
}


bool
FootstepNavigation::performable(const humanoid_nav_msgs::msg::StepTarget& footstep)
{
  float step_x = footstep.pose.x;
  float step_y = footstep.pose.y;
  float step_theta = footstep.pose.theta;

  if (footstep.leg == humanoid_nav_msgs::msg::StepTarget::RIGHT)
  {
    step_y = -step_y;
    step_theta = -step_theta;
  }

  if (step_x + FLOAT_CMP_THR > ivMaxStepX ||
      step_x - FLOAT_CMP_THR < ivMaxInvStepX)
    return false;
  if (step_y + FLOAT_CMP_THR > ivMaxStepY ||
      step_y - FLOAT_CMP_THR < ivMaxInvStepY)
    return false;
  if (step_theta + FLOAT_CMP_THR > ivMaxStepTheta ||
      step_theta - FLOAT_CMP_THR < ivMaxInvStepTheta)
    return false;

  return performable(step_x, step_y);
}


bool
FootstepNavigation::performable(float step_x, float step_y)
{
  int cn = 0;

  // loop through all ivStepRange of the polygon
  for(unsigned int i = 0; i < ivStepRange.size() - 1; ++i)
  {
    if ((ivStepRange[i].second <= step_y &&
    	 ivStepRange[i + 1].second > step_y) ||
        (ivStepRange[i].second >= step_y &&
         ivStepRange[i + 1].second < step_y))
    {
      float vt = (float)(step_y - ivStepRange[i].second) /
        (ivStepRange[i + 1].second - ivStepRange[i].second);
      if (step_x <
          ivStepRange[i].first + vt *
            (ivStepRange[i + 1].first - ivStepRange[i].first))
      {
        ++cn;
      }
    }
  }
  return cn & 1;
}
}
