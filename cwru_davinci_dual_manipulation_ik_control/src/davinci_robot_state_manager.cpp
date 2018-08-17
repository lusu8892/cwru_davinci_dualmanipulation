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
 * Description: A class to manage robot states. This class allows to manage robot states,
 * making it easy to update a state from the published robot state or other states/trajectories.
 */

#include <string>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_robot_state_manager.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#define CLASS_LOGNAME "DavinciRobotStateManager"
#define CLASS_NAMESPACE "DavinciRobotStateManager::"

DavinciRobotStateManager::DavinciRobotStateManager(const moveit::core::RobotModelConstPtr &robot_model,
                                                   const std::string &joint_states_topic,
                                                   const std::string &full_robot_group)
  : full_robot_name_(full_robot_group)
{
  const boost::shared_ptr<tf::Transformer> tf(new tf::Transformer());
  current_state_monitor_.reset(new planning_scene_monitor::CurrentStateMonitor(robot_model,tf));
  current_state_monitor_->startStateMonitor(joint_states_topic);
  updateCurrentState(full_robot_name_);
}

bool DavinciRobotStateManager::resetRobotState(const moveit::core::RobotStatePtr &rs,
                                               const std::string &group,
                                               std::mutex &rs_mutex) const
{
  if (!updateCurrentState(group))
    return false;
  std::unique_lock<std::mutex> lck1(rs_mutex, std::defer_lock);
  std::unique_lock<std::mutex> lck2(cs_mutex_, std::defer_lock);

  // check that we are using the same robot
  assert(current_state_ ->getRobotModel()->getName() == rs->getRobotModel()->getName());

  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName());

  for(int i = 0; i < rs->getVariableCount(); i++)
  {
    rs->setVariablePosition(i, current_state_->getVariablePosition(i));
  }

  return true;
}

bool DavinciRobotStateManager::resetRobotState(const moveit::core::RobotStatePtr &rs,
                                                const std::string &group,
                                                std::mutex &rs_mutex,
                                                const moveit_msgs::RobotTrajectory &traj) const
{
  if(traj.joint_trajectory.points.empty())
    return true;

  std::unique_lock<std::mutex> lck1(rs_mutex, std::defer_lock);
  std::unique_lock<std::mutex> lck2(cs_mutex_, std::defer_lock);

  ROS_INFO_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__ << " : resetting " << rs->getRobotModel()->getName()
                                                       << " with a trajectory for group " << group);

  // can't use the "same robot" check here, so making it more complex (and more time-consuming...)
  assert(rs->getJointModelGroup(group)->getVariableCount() == traj.joint_trajectory.joint_names.size());
  for(int i=0; i<rs->getJointModelGroup(group)->getVariableCount(); ++i)
  {
    assert(rs->getJointModelGroup(group)->getVariableNames().at(i).compare( traj.joint_trajectory.joint_names.at(i) ) == 0);
  }

  //NOTE: robot_traj, built on robot_model, contains the full robot; trajectory, instead, is only for the group joints
  robot_trajectory::RobotTrajectory robot_traj(rs->getRobotModel(), group);
  robot_traj.setRobotTrajectoryMsg(*rs, traj);

  for(int i=0; i<rs->getVariableCount(); i++)
    rs->setVariablePosition(i,robot_traj.getLastWayPoint().getVariablePosition(i));

  return true;
}


bool DavinciRobotStateManager::updateCurrentState(const std::string& group, double wait_seconds) const
{
  std::unique_lock<std::mutex> ul(cs_mutex_);
  if (!current_state_monitor_ || !current_state_monitor_->isActive())
  {
    ROS_ERROR_STREAM_NAMED(CLASS_LOGNAME,CLASS_NAMESPACE << __func__ << " : Unable to get current robot state");
    return false;
  }

  if (!current_state_monitor_->waitForCurrentState(group, wait_seconds))
    ROS_WARN_STREAM_NAMED(CLASS_LOGNAME, CLASS_NAMESPACE << __func__
                                                         << " : Joint values for monitored state are requested but the full state is not known");

  current_state_ = current_state_monitor_->getCurrentState();
  return true;
}

moveit::core::RobotStatePtr DavinciRobotStateManager::getRobotStateCopy() const
{
  return moveit::core::RobotStatePtr(new moveit::core::RobotState(*current_state_));
}

