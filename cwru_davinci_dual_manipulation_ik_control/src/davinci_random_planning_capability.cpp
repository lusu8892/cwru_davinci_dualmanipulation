/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Su Lu <sxl924@case.edu>
 * Description: Here define stuff which is needed from various possible planning capabilities,
 * that will be specialized later on
 */


#include <cwru_davinci_dual_manipulation_ik_control/davinci_random_planning_capability.h>
#include <cwru_davinci_dual_manipulation_shared/parsing_utils.h>


#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>

#define CLASS_NAMESPACE "ikControl::randomPlanningCapability::"
#define CLASS_LOGNAME "ikControl::randomPlanningCapability"
#define DEFAULT_MAX_PLANNING_ATTEMPTS 1
#define HIGH_UNGRASP_WP_IF_COLLIDING 0.1
#define MAX_REPLAN 10
#define TABLE_WP_HEIGHT 0.1
#define ALLOWED_JOINT_JUMP 0.5 // allow at most ALLOWED_JOINT_JUMP rads jump per joint between two successive points in a trajectory

#define REFACTORED_OUT 1

using namespace cwru_davinci_dual_manipulation::cwru_davinci_ik_control;

DavinciRandomPlanningCapability::DavinciRandomPlanningCapability(DavinciSharedIKMemory &sikm, const ros::NodeHandle &nh)
  : sikm_(sikm), nh_(nh)
{
  reset();
}

DavinciRandomPlanningCapability::~DavinciRandomPlanningCapability()
{

}

bool DavinciRandomPlanningCapability::canPerformCapability(const IkControlCapabilities &ik_capability) const
{
  if ((ik_capability == IkControlCapabilities::PLAN) ||
      (ik_capability == IkControlCapabilities::PLAN_NO_COLLISION) ||
      (ik_capability == IkControlCapabilities::PLAN_BEST_EFFORT) ||
      (ik_capability == IkControlCapabilities::PLAN_CLOSE_BEST_EFFORT))
    return true;

  return false;
}

void DavinciRandomPlanningCapability::reset()
{
  std::unique_lock<std::mutex> ul(sikm_.m);
  parseParameters(*(sikm_.ik_control_params));

  setParameterDependentVariables();

  busy_.store(false);
}


void DavinciRandomPlanningCapability::parseParameters(XmlRpc::XmlRpcValue &params)
{
  ROS_ASSERT(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  // planner parameters
  if (params.hasMember("motion_planner"))
  {
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], planner_id_, "planner_id");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], planning_time_,
                                                                 "planning_time");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], backup_planner_id_,
                                                                 "backup_planner_id");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], backup_planning_time_,
                                                                 "backup_planning_time");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], max_planning_attempts_,
                                                                 "max_planning_attempts");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"],
                                                                 backup_max_planning_attempts_,
                                                                 "backup_max_planning_attempts");

    if (max_planning_attempts_ <= 0) max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;
    if (backup_max_planning_attempts_ <= 0) backup_max_planning_attempts_ = DEFAULT_MAX_PLANNING_ATTEMPTS;

    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], goal_position_tolerance_,
                                                                 "goal_position_tolerance");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], goal_orientation_tolerance_,
                                                                 "goal_orientation_tolerance");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], goal_joint_tolerance_,
                                                                 "goal_joint_tolerance");
    cwru_davinci_dual_manipulation::shared::parseSingleParameter(params["motion_planner"], ws_bounds_,
                                                                 "workspace_bounds", 6);
  }
}

void DavinciRandomPlanningCapability::setParameterDependentVariables()
{
  robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
    new robot_model_loader::RobotModelLoader("robot_description"));

  robot_model_ = robot_model_loader_->getModel();

  ros::NodeHandle move_group_node("move_group");
  ros::NodeHandle private_nh("~");

  if (nh_.hasParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction"))
  {
    double jiggle_fraction;
    nh_.getParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction", jiggle_fraction);
    private_nh.setParam("jiggle_fraction", jiggle_fraction);
  }
  if (nh_.hasParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts"))
  {
    double max_sampling_attempts;
    nh_.getParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts", max_sampling_attempts);
    private_nh.setParam("max_sampling_attempts", max_sampling_attempts);
  }

  target_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));

  pipeline_ = planning_pipeline::PlanningPipelinePtr(
    new planning_pipeline::PlanningPipeline(target_rs_->getRobotModel(), move_group_node, "planning_plugin",
                                            "request_adapters"));

  MotionPlanReq_.allowed_planning_time = planning_time_;
  MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
  MotionPlanReq_.planner_id = planner_id_;
  MotionPlanReq_.workspace_parameters.header.frame_id = robot_model_->getRootLinkName();

  geometry_msgs::Vector3 min_corner, max_corner;
  min_corner.x = ws_bounds_.at(0);
  min_corner.y = ws_bounds_.at(1);
  min_corner.z = ws_bounds_.at(2);
  max_corner.x = ws_bounds_.at(3);
  max_corner.y = ws_bounds_.at(4);
  max_corner.z = ws_bounds_.at(5);
  MotionPlanReq_.workspace_parameters.min_corner = min_corner;
  MotionPlanReq_.workspace_parameters.max_corner = max_corner;
}

void DavinciRandomPlanningCapability::performRequest(cwru_davinci_dual_manipulation_shared::ik_serviceRequest req)
{
  // tell the interface I'm busy
  bool I_am_busy = busy_.exchange(true);
  if (I_am_busy)
    return;

  ros::Time planning_start = ros::Time::now();
  std::string b = "\033[0;34m";
  std::string n = "\033[0m";

  moveit::planning_interface::MoveGroupInterface *localMoveGroup;
  std::string group_name;
  std::string group_name_true;
  std::map<std::string, IkTarget> local_targets;
  bool exists = sikm_.groupManager->getGroupInSRDF(req.ee_name, group_name);

  assert(exists);
  IkControlCapabilities local_capability = capability_.from_name.at(req.command);
  bool check_collisions = (local_capability != IkControlCapabilities::PLAN_NO_COLLISION);

  map_mutex_.lock();  // mutex is intended protect variable targets_

  // in case I'm looking for a single target
  if (targets_.count(req.ee_name) != 0)
  {
    local_targets[req.ee_name] = targets_.at(req.ee_name);
    targets_.erase(req.ee_name);
  }
    // here, I'm looking for a possible composition
  else if (sikm_.groupManager->isTree(req.ee_name))
  {
    for (auto chain:sikm_.groupManager->getTreeComposition(req.ee_name))
      if (targets_.count(chain))
      {
        local_targets[chain] = targets_.at(chain);
        targets_.erase(chain);
      }
  }
    // No target to be set...!
  else
  {
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : no target needs to be set...");
  }

  map_mutex_.unlock();

  if (group_name == "full_robot")
  {
    std::vector<std::string> targets;
    for (auto &t:local_targets)
      targets.push_back(t.first);
    group_name_true = sikm_.groupManager->findGroupName(targets);
  }
  else
    group_name_true = group_name;

  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,
                        CLASS_NAMESPACE << __func__ << " : Planning for group " << group_name << " (" << group_name_true
                                        << ")");

  moveit::planning_interface::MoveItErrorCode error_code;

  plan_response_.seq = req.seq;
  plan_response_.group_name = req.group_name;
  moveit::planning_interface::MoveGroupInterface::Plan movePlan;

  // add a check for generated plans: if the jump is too high, try replanning
  bool need_replan = true;
  int count = 0;
  while (need_replan && (count++ < MAX_REPLAN))
  {
    double plan_time;
    ros::Time tmp;
    // wait for the execution to be initialized -
    // sleep 5ms if an execution function has been called but has not passed to actual execution yet

    while (!sikm_.getNextTrajectoryEndTime(tmp))
    {
      usleep(5000);
    }

    ros::Duration residual_move_time = tmp - ros::Time::now();
    plan_time = std::max(planning_time_, residual_move_time.toSec());
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : will use " << plan_time
                                                         << "s planning time, max between default " << planning_time_
                                                         << "s and residual movemenent time "
                                                         << residual_move_time.toSec() << "s");

    planning_interface::MotionPlanResponse MotionPlanRes;

    MotionPlanReq_.allowed_planning_time = plan_time;
    MotionPlanReq_.group_name = group_name_true;
    MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
    MotionPlanReq_.planner_id = planner_id_;

    bool copy_attached_bodies(check_collisions);

    //TO
    moveit::core::robotStateToRobotStateMsg(sikm_.getPlanningRobotState(), MotionPlanReq_.start_state);
    MotionPlanReq_.start_state.attached_collision_objects.clear();

    if (copy_attached_bodies)
    {
      auto aco = sikm_.sceneObjectManager->getAttachedCollisionObjects();
      MotionPlanReq_.start_state.attached_collision_objects.insert(
        MotionPlanReq_.start_state.attached_collision_objects.begin(), aco->begin(), aco->end());
    }

#if DEBUG > 1
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : debugging attachedn collision objects...\n");
    for(auto attObject:MotionPlanReq_.start_state.attached_collision_objects)
    {
      std::cout << attObject.object.id << ": touch links are > | " << std::endl;
      for(auto link:attObject.touch_links)
        std::cout << link << " | ";
      std::cout << std::endl;
      std::cout << "waiting for input..." << std::endl;;
      char y;
      std::cin >> y;
    }
#endif

    MotionPlanReq_.start_state.is_diff = false;

    //
    MotionPlanReq_.goal_constraints.clear();

    moveit_msgs::Constraints empty_constr;
    MotionPlanReq_.path_constraints = empty_constr;

    moveit_msgs::TrajectoryConstraints empty_traj_constr;
    MotionPlanReq_.trajectory_constraints = empty_traj_constr;

    bool planRes = buildMotionPlanRequest(MotionPlanReq_, local_targets, local_capability);

    if (!planRes)
      ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,
                             CLASS_NAMESPACE << __func__ << " : unable to obtain motion planning request!!!");
    else
    {
      {
        const planning_scene::PlanningSceneConstPtr ps = sikm_.sceneObjectManager->lockAndGetReadOnlyPlanningScene();
        planRes = pipeline_->generatePlan(ps, MotionPlanReq_, MotionPlanRes);
      }

      if (planRes)
      {
        moveit_msgs::MotionPlanResponse msg;
        MotionPlanRes.getMessage(msg);
        movePlan.trajectory_ = msg.trajectory;
      }
    }

    error_code = MotionPlanRes.error_code_;
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",
                          b << CLASS_NAMESPACE << __func__ << " : This planning (" << planner_id_ << ") took [s]: "
                            << (ros::Time::now() - planning_start).toSec() << n);

    if (error_code.val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_WARN_STREAM_NAMED(CLASS_LOGNAME,
                            CLASS_NAMESPACE << __func__ << " : unable to plan with \'" << backup_planner_id_
                                            << "\' with timeout of " << plan_time << "s, trying last time with \'"
                                            << backup_planner_id_ << "\' and timeout of " << backup_planning_time_
                                            << "s");

      MotionPlanReq_.planner_id = backup_planner_id_;
      MotionPlanReq_.allowed_planning_time = backup_planning_time_;
      MotionPlanReq_.num_planning_attempts = backup_max_planning_attempts_;
      {
        const planning_scene::PlanningSceneConstPtr ps = sikm_.sceneObjectManager->lockAndGetReadOnlyPlanningScene();
        planRes = pipeline_->generatePlan(ps, MotionPlanReq_, MotionPlanRes);
      }
      if (planRes)
      {
        moveit_msgs::MotionPlanResponse msg;
        MotionPlanRes.getMessage(msg);
        movePlan.trajectory_ = msg.trajectory;
      }

      error_code = MotionPlanRes.error_code_;
      ROS_INFO_STREAM_NAMED(CLASS_LOGNAME + "_TIMING",
                            b << CLASS_NAMESPACE << __func__ << " : This planning (" << backup_planner_id_
                              << ") took [s]: " << (ros::Time::now() - planning_start).toSec() << n);

      MotionPlanReq_.planner_id = planner_id_;
      MotionPlanReq_.allowed_planning_time = planning_time_;
      MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
    }

    if (!movePlan.trajectory_.joint_trajectory.points.empty())
    {
      // TODO
      need_replan = !checkTrajectoryContinuity(movePlan.trajectory_, ALLOWED_JOINT_JUMP);
    }
    else
      need_replan = false;

    // make sure any wrong plan is erased
    if (need_replan)
      movePlan.trajectory_ = moveit_msgs::RobotTrajectory();
  }


  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : movePlan traj size: "
                                                       << movePlan.trajectory_.joint_trajectory.points.size()
                                                       << std::endl);
  for (int i = 0; i < movePlan.trajectory_.joint_trajectory.points.size(); i++)
  {
    ROS_DEBUG_STREAM(movePlan.trajectory_.joint_trajectory.points.at(i) << std::endl);
  }

  ROS_DEBUG_STREAM("pos [x y z]: " << req.ee_pose.at(0).position.x << " "
                                   << req.ee_pose.at(0).position.y << " "
                                   << req.ee_pose.at(0).position.z << std::endl);
  ROS_DEBUG_STREAM(
    "orient [x y z w]: " << req.ee_pose.at(0).orientation.x << " " << req.ee_pose.at(0).orientation.y << " "
                         << req.ee_pose.at(0).orientation.z << " " << req.ee_pose.at(0).orientation.w << std::endl);

  if (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    plan_response_.data = "done";

    moveit_msgs::RobotTrajectory traj = movePlan.trajectory_;
    sikm_.swapTrajectory(req.ee_name, traj);
  }
  else
  {
    plan_response_.data = "error";
  }

  // update planning_init_rs_ with trajectory last waypoint
  sikm_.resetPlanningRobotState(group_name_true, movePlan.trajectory_);
  busy_.store(false);
  return;
}

bool DavinciRandomPlanningCapability::isComplete()
{
  return !busy_.load();
}

bool DavinciRandomPlanningCapability::canRun()
{
  return !busy_.load();
}

bool DavinciRandomPlanningCapability::getResults(cwru_davinci_dual_manipulation_shared::ik_response &res)
{
  // fill response message
  if (busy_.load())
    return false;

  res = plan_response_;
  return true;
}

bool DavinciRandomPlanningCapability::buildMotionPlanRequest(moveit_msgs::MotionPlanRequest &req,
                                                             const std::map<std::string, IkTarget> &targets,
                                                             IkControlCapabilities plan_type)
{
  // TODO: define a set of tolerances depending on the capability (these will be parameterized from outside...!)
  std::map<IkControlCapabilities, double> position_tolerance;
  std::map<IkControlCapabilities, double> orientation_tolerance;

  position_tolerance[IkControlCapabilities::PLAN] = goal_position_tolerance_;
  position_tolerance[IkControlCapabilities::PLAN_BEST_EFFORT] = 10 * goal_position_tolerance_;
  position_tolerance[IkControlCapabilities::PLAN_NO_COLLISION] = goal_position_tolerance_;
  position_tolerance[IkControlCapabilities::PLAN_CLOSE_BEST_EFFORT] = 10 * goal_position_tolerance_;
  orientation_tolerance[IkControlCapabilities::PLAN] = goal_orientation_tolerance_;
  orientation_tolerance[IkControlCapabilities::PLAN_BEST_EFFORT] = 5 * goal_orientation_tolerance_;
  orientation_tolerance[IkControlCapabilities::PLAN_NO_COLLISION] = goal_orientation_tolerance_;
  orientation_tolerance[IkControlCapabilities::PLAN_CLOSE_BEST_EFFORT] = 50 * goal_orientation_tolerance_;

  if (!position_tolerance.count(plan_type))
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : Unknown plan_type!!! returning...");
    return false;
  }

  double pos_tol = position_tolerance.at(plan_type);
  double orient_tol = orientation_tolerance.at(plan_type);
  double joint_tol = goal_joint_tolerance_;

  bool position_only = (plan_type == IkControlCapabilities::PLAN_BEST_EFFORT ||
                        plan_type == IkControlCapabilities::PLAN_CLOSE_BEST_EFFORT);

  if (position_only)
  {
    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__
                                                         << " : planning position_only > increasing the position tolerance from "
                                                         << goal_position_tolerance_ << " to " << pos_tol);

    ROS_INFO_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__
                                                         << " : planning position_only > increasing the orientation tolerance from "
                                                         << goal_orientation_tolerance_ << " to " << orient_tol);
  }

  bool is_close = (plan_type == IkControlCapabilities::PLAN_NO_COLLISION ||
                   plan_type == IkControlCapabilities::PLAN_CLOSE_BEST_EFFORT);

  moveit_msgs::Constraints c;

  for (auto target_it:targets)
  {
    IkTarget& target(target_it.second);
    moveit_msgs::Constraints c_tmp;

    std::string group_name;
    bool exists = sikm_.groupManager->getGroupInSRDF(target.ee_name, group_name);

    assert(exists);
    const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(group_name);

    if(target.type == IkTargetType::NAMED_TARGET)
    {
      setTarget(target.ee_name, target.target_name);

      std::unique_lock<std::mutex> ul(robotState_mutex_);
      c_tmp = kinematic_constraints::constructGoalConstraints(*target_rs_,jmg,joint_tol);
    }
    



  }
}



