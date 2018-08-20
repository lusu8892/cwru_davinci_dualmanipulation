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

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_RANDOM_PLANNING_CAPABILITY_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_RANDOM_PLANNING_CAPABILITY_H

#include <atomic>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_generic_planning_capability.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace cwru_davinci_dual_manipulation
{
namespace cwru_davinci_ik_control
{

class DavinciRandomPlanningCapability : public DavinciGenericPlanningCapability
{
public:
  DavinciRandomPlanningCapability(DavinciSharedIKMemory& sikm, const ros::NodeHandle& nh = ros::NodeHandle());
  virtual ~DavinciRandomPlanningCapability();
  virtual bool isComplete();
  virtual void performRequest(cwru_davinci_dual_manipulation_shared::ik_serviceRequest req);
  virtual bool getResults(cwru_davinci_dual_manipulation_shared::ik_response& res);
  virtual bool canRun();
  virtual bool canPerformCapability(const IkControlCapabilities& ik_capability) const;
  virtual void reset();

  /**
   * @brief add a target to the internal targets list
   *
   * @param req the same req from the @e ik_service
   */
  void add_target(const cwru_davinci_dual_manipulation_shared::ik_service::Request& req);

private:
  DavinciSharedIKMemory& sikm_;
  const DavinciIKControlCapability capability_;

  std::mutex map_mutex_;
  std::mutex robotState_mutex_;

  // keep an history of the required targets
  std::map<std::string, IkTarget> targets_;

  // MoveIt! variables
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr target_rs_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_pipline::PlannigPipeLinePtr pipeline_;
  planning_interface::MotionPlanRequest MotionPlanReq_;

  // ros variables
  ros::NodeHandle nh_;

  // planner parameters
  std::string planner_id_;
  double planning_time_;
  std::string backup_planner_id_;
  double backup_planning_time_;
  int max_planning_attempts_;
  int backup_max_planning_attempts_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double goal_joint_tolerance_;
  std::vector<double> ws_bounds_;

  // interface and results variables
  std::atomic_bool busy_;
  cwru_davinci_dual_manipulation_shared::shared::ik_response plan_response_;

private:

  /**
   * @brief update a motionPlan request with a new target, considering the type of plan that will follow
   * @param req
   * @param targets
   * @param plan_type
   * @return
   */
  bool buildMotionPlanRequest(moveit_msgs::MotionPlanRequest& req,
                              const std::map< std::string, IkTarget >& targets,
                              IkControlCapabilities plan_type);

  /**
   * @brief utility function to parse parameters from the parameter server
   * @param params all useful params got from the parameter server
   * @return void
   */
  void parseParameters(XmlRpc::XmlRpcValue& params);

  /**
   * @brief utility function to set class variables which depend on parameters
   * @return void
   */
  void setParameterDependentVariables();

  /**
   * @brief set the target robot state of the eef @param ee_name to the target specified in the SRDF
   *        with name @param named_target
   * @param ee_name the end-effector we want to set a target for
   * @param named_target the target name as specified in the SRDF
   * @return
   */
  bool setTarget(std::string ee_name, std::string named_target);

};
}
}
#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_RANDOM_PLANNING_CAPABILITY_H
