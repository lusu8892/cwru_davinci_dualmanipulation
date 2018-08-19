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
 * Description: The management of shared memory between threads.
 * A structure to share resources across implemented capabilities: plan, move, grasp, ...
 */

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_SHARED_IK_MEMORY_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_SHARED_IK_MEMORY_H

#include <mutex>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_group_structure_manager.h>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_robot_controller_interface.h>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_robot_state_manager.h>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_scene_object_manager.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <XmlRpcValue.h>

namespace cwru_davinci_dual_manipulation
{
namespace cwru_davinci_ik_control
{
class DavinciSharedIKMemory
{
public:
  DavinciSharedIKMemory(XmlRpc::XmlRpcValue& params, const ros::NodeHandle& nh);
  ~DavinciSharedIKMemory() {};

  void reset();

  /**
   * @brief Automically get/set the planned trajectory for the specified group.
   *
   * @param group The name of the group to use for swapping the trajectory
   *
   * @param traj The trajectory to use for swapping
   *
   * @return False if the group did not exist, true otherwise (but @p traj can be empty)
   */
  bool swapTrajectory(const std::string& group, moveit_msgs::RobotTrajectory& traj);

  /**
   * @brief Inform that a trajectory execution is pending and planning should not be initialized until it started.
   *        To query this property, call @fn getNextTrajectoyEndTime
   *
   * @return false if a trajectory execution was already pending, true otherwise.
   */
  bool setPendingTrajectoryExecution();

  /**
   * @brief Set the (relative) duration of the next trajectory execution,
   *        to give more time to planning. Can be called only if a trajectory execution is pending.
   *
   * @param dt
   *
   * @return false if a trajectory execution was not pending, true otherwise.
   */
  bool setNextTrajectoryRelativeEndTime(const ros::Duration& dt);

  /**
   * @brief Get trajectory execution end time.
   *        Planning should not be initialized until there is no trajectory execution pending.
   * @param end_t The time at which the trajectory execution is supposed to end
   *
   * @return false if a trajectory execution is still pending, true otherwise
   */
  bool getNextTrajectoryEndTime(ros::Time& end_t);

  /**
   * @brief Get a copy of the robot state which can be used for planning
   */
  moveit::core::RobotState getPlanningRobotState();

  /**
   * @brief Reset the joint group @p group in the robot state used for planning
   *
   * @param group The group to reset in the planning robot state
   *
   * @return true on success
   */
  bool resetPlanningRobotState(const std::string& group);

  /**
   * @brief Reset the joint group @p group in the robot state used for planning using the last waypoint in the trajectory @p traj
   *
   * @param group The group to reset in the planning robot state
   *
   * @param traj The trajectory to use in resetting the robot state
   *
   * @return true on success
   */
  bool resetPlanningRobotState(const std::string& group, const moveit_msgs::RobotTrajectory& traj);

public:
  std::mutex m;
  XmlRpc::XmlRpcValue* ik_control_params;

  // manage robot group structure
  std::unique_ptr<const DavinciGroupStructureManager> groupManager;
  // manage robot controllers
//  std::unique_ptr<const DavinciRobotControllerInterface> robotController;
  // manage robot states
  std::unique_ptr<const DavinciRobotStateManager> robotStateManager;
  // managing the objects in the scene
  std::unique_ptr<DavinciSceneObjectManager> sceneObjectManager;

private:
  // MoveIt! variables
  moveit::core::RobotModelPtr robot_model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  // utility variables
  std::string joint_states_;
  std::string robot_description_;
  std::string full_robot_group_;

  // share the robot state to use for next planning
  std::mutex robotState_mutex_;
  moveit::core::RobotStatePtr planning_init_rs_;

  // trajectory execution expected end-time
  std::mutex end_time_mutex_;
  ros::Time movement_end_time_;

  // share the motion plans among planning/control capabilities
  std::mutex movePlans_mutex_;
  std::map<std::string, moveit::planning_interface::MoveGroupInterface::Plan> movePlans_;

private:
  /**
   * @brief Utility function to parse parameters from the parameter server
   *
   * @param params params got from the parameter server
   */
  void parseParameters(XmlRpc::XmlRpcValue& params);
};

}
}


#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_SHARED_IK_MEMORY_H
