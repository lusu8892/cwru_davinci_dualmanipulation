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

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_ROBOT_STATE_MANAGER_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_ROBOT_STATE_MANAGER_H

#include <map>
#include <mutex>
#include <XmlRpcValue.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene_monitor/current_state_monitor.h>

class DavinciRobotStateManager
{
  /**
   * @brief Initialize a robot state manager
   *
   * @param robot_model A robot model to construct the state monitor
   * @param joint_states_topic The topic to use for listening to joint states
   * @param full_robot_group The SRDF group name of the full robot
   * @return
   */
  DavinciRobotStateManager(const moveit::core::RobotModelConstPtr &robot_model,
                           const std::string &joint_states_topic,
                           const std::string &full_robot_group);

  ~DavinciRobotStateManager() {}

  /**
   * @brief utility to reset the state of the parameter @p rs to the current robot state
   *
   * @param rs the robot state to reset
   * @param group the SRDF group to reset
   * @param rs_mutex the robot_state mutex to use to access the robot state
   *
   * @return true on success
   */
  bool resetRobotState(const moveit::core::RobotStatePtr &rs,
                       const std::string &group,
                       std::mutex &rs_mutex) const;

  /**
   * @brief utility to reset the state of the parameter @p rs to the final position in the @p traj trajectory
   *
   * @param rs the robot state to reset
   * @param group the SRDF group to reset
   * @param rs_mutex the robot_state mutex to use to access the robot state
   * @param traj robot trajectory of which to use the last waypoint to update @p rs
   *
   * @return true on success
   */
  bool reset_robot_state(const moveit::core::RobotStatePtr &rs,
                         const std::string &group,
                         std::mutex &rs_mutex,
                         const moveit_msgs::RobotTrajectory &traj) const;

  /**
   * @brief Return a copy of the internal robot state
   */
  moveit::core::RobotStatePtr getRobotStateCopy() const;

private:
  mutable robot_state::RobotStatePtr current_state_;
  planning_scene_monitor::CurrentStateMonitorPtr current_state_monitor_;
  mutable std::mutex cs_mutex_;
  const std::string full_robot_name_;

private:

  /**
   * @brief Read updated robot state from server.
   *
   * @param group the SRDF group to read the state of
   * @param wait_seconds time to wait (at most) for the updated state
   *
   * @return True on success
   */
  bool updateCurrentState(const std::string& group, double wait_seconds = 1.0) const;
};

#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_ROBOT_STATE_MANAGER_H
