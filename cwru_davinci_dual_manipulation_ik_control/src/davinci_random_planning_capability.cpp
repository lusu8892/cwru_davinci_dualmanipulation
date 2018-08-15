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


#include <cwru_davinci_dual_manipulation_ik_control/random_planning_capability.h>


#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group.h>
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

DavinciRandomPlanningCapability::DavinciRandomPlanningCapability(shared_ik_memory &sikm, const ros::NodeHandle &nh)
  : sikm_(sikm), nh_(nh)
{
  reset();
}

DavinciRandomPlanningCapability::~DavinciRandomPlanningCapability()
{

}

bool DavinciRandomPlanningCapability::canPerformCapability(const IKControlCapabilities &ik_capability) const
{
  if ((ik_capability == IKControlCapabilities::PLAN) ||
      (ik_capability == IKControlCapabilities::PLAN_NO_COLLISION) ||
      (ik_capability == IKControlCapabilities::PLAN_BEST_EFFORT) ||
      (ik_capability == IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT))
    return true;

  return false;
}

void DavinciRandomPlanningCapability::reset()
{
  std::unique_lock <std::mutex> ul(sikm.m);
  parseParameters(*(sikm.ik_control_parames));

  setParameterDependentVariables();

  busy.store(false);
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

bool DavinciRandomPlanningCapability::setParameterDependentVariables()
{
  robot_model_loader_ = robot_model_loader::RobotModelLoaderPtr(
    new robot_model_loader::RobotModelLoader("robot_description"));

  robot_model_ = robot_model_loader_->getModel();

  ros::NodeHandle move_group_node("move_group");
  ros::NodeHandle private_nh("~");

  if(node.hasParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction"))
  {
    double jiggle_fraction;
    node.getParam("ik_control_parameters/fix_start_state_collision/jiggle_fraction",jiggle_fraction);
    private_nh.setParam("jiggle_fraction",jiggle_fraction);
  }
  if(node.hasParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts"))
  {
    double max_sampling_attempts;
    node.getParam("ik_control_parameters/fix_start_state_collision/max_sampling_attempts",max_sampling_attempts);
    private_nh.setParam("max_sampling_attempts",max_sampling_attempts);
  }

  target_rs_ = moveit::core::RobotStatePtr(new moveit::core::RobotState(robot_model_));

  pipeline_ = planning_pipeline::PlanningPipelinePtr(
    new planning_pipeline::PlanningPipeline(target_rs_->getRobotModel(), move_group_node, "planning_plugin",
                                            "request_adapters"));

  MotionPlanReq_.allowed_planning_time = planning_time_;
  MotionPlanReq_.num_planning_attempts = max_planning_attempts_;
  MotionPlanReq_.planner_id = planner_id_;
  MotionPlanReq_.workspace_parameters.header.frame_id = robot_model_->getRootLinkName();

  geometry_msgs::Vector3 min_corner,max_corner;
  min_corner.x = ws_bounds_.at(0); min_corner.y = ws_bounds_.at(1); min_corner.z = ws_bounds_.at(2);
  max_corner.x = ws_bounds_.at(3); max_corner.y = ws_bounds_.at(4); max_corner.z = ws_bounds_.at(5);
  MotionPlanReq_.workspace_parameters.min_corner = min_corner;
  MotionPlanReq_.workspace_parameters.max_corner = max_corner;
}



