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
 * Description: This is a class that is used from the ros_server to
 * perform a desired ik using a dedicated thread (one for each end effector).
 */

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H

#include <cwru_davinci_dual_manipulation_shared/ik_service.h>
#include <cwru_davinci_dual_manipulation_shared/scene_object_service.h>
#include <cwru_davinci_dual_manipulation_shared/ik_control_capabilities.h>

#include <thread>
#include <XmlRpcValue.h>
#include <mutex>

#include <std_msgs/String.h>
// capabilities definition
#include <cwru_davinci_dual_manipulation_ik_control/ik_check_capability.h>
#include <cwru_davinci_dual_manipulation_ik_control/davinci_random_planning_capability.h>
#include <cwru_davinci_dual_manipulation_ik_control/trajectory_execution_capability.h>
#include <cwru_davinci_dual_manipulation_ik_control/grasping_capability.h>

namespace cwru_davinci_dual_manipulation
{
namespace cwru_davinci_ik_control
{

/**
 * @brief Class to manage a single object in an atomic manner, making sure that it gets assigned a desired value
 * (when this object goes out of scope, it makes the assignment using the appropriate LOCK-able variable)
 *
 * @p LOCK has to be lockable, @p T has to provide a copy constructor and the assignment operator (operator=)
 */
template<typename LOCK, typename T>
class ObjectLocker
{
  const LOCK &m_;
  T &flag_;
  const T result_;
public:
  ObjectLocker(const LOCK &m, T &flag, const T &result) : m_(m), flag_(flag), result_(result)
  {}

  ~ObjectLocker()
  {
    std::unique_lock<LOCK>(m_);
    flag_ = result_;
  }
};

class DavinciIkControl
{
public:
  DavinciIkControl();

  ~DavinciIkControl()
  {};

  /**
   * interface function to perform the @e ik_service
   * @param req the req from the @e ik_service
   * @return
   */
  bool performIk(cwru_davinci_dual_manipulation_shared::ik_service::Request &req);

  bool manageObject(cwru_davinci_dual_manipulation_shared::scene_object_service::Request &req);

private:
  // manage IK requests
  std::unique_ptr<DavinciIkCheckCapability> ik_check_legacy_;

  // MoveIt! variables
  moveit::core::RobotModelPtr robot_model_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;

  // ros variables
  ros::NodeHandle node_;
  std::map<ik_control_capabilities, ros::Publisher> hand_pub_;

  // utility variables
  std::vector<std::unique_ptr<std::thread>> used_threads_;
  std::map<ik_control_capability_types, std::map<std::string, bool>> busy;
  const ik_control_capability capabilities_;
  std::mutex map_mutex_; // busy
  std::mutex ikCheck_mutex_;
  std::string joint_states_;
  std::string robot_description_;
  std::string full_robot_group_;

  // managing external parameters
  XmlRpc::XmlRpcValue ik_control_params_;

  double hand_max_velocity_;   // maximum hand velocity : avg is 2.0, closes completely [0.0->1.0] in half a second
  double epsilon_;            // IK tolerance used by KDLKinematicsPlugin

  // shared parameters between capabilities
  std::unique_ptr<shared_ik_memory> sikm_;
  // capabilities
  std::unique_ptr<DavinciRandomPlanningCapability> rndmPlan_;
  std::unique_ptr<DavinciGroupStructureManager> trajExecute_;
  std::unique_ptr<DavinciGraspingCapability> graspPlanExecute_;

  /**
   * @brief utility function to parse parameters from the parameter server
   *
   * @param params
   *   all useful params got from the parameter server
   * @return void
   */
  void parseParameters(XmlRpc::XmlRpcValue &params);

  /**
   * @brief utility function to set all class parameters to their default value
   *
   * @return void
   */
  void setDefaultParameters();

  /**
   * @brief utility function to set class variables which depend on parameters
   *
   * @return void
   */
  void setParameterDependentVariables();

  /**
   * @brief this is the thread body to perform IK feasibility check (no collision considered)
   *
   * @param req
   *   the same req from the @e ik_service
   * @return void
   */
  void ikCheckThread(cwru_davinci_dual_manipulation_shared::ik_service::Request req);

  /**
   * @brief this is the thread body to perform trajectory generation
   *
   * @param req
   *   the same req from the @e ik_service
   *
   * @return void
   */
  void planningThread(cwru_davinci_dual_manipulation_shared::ik_service::Request req);

  /**
   * @brief execute last planned path
   *
   * @return void
   */
  void executePlan(cwru_davinci_dual_manipulation_shared::ik_service::Request req);


  /**
   * @brief function to move a group to its home position and to open the hand
   *
   * @param ee_name
   *   which end-effector bring back home
   * @return void
   */
  void simpleHoming(cwru_davinci_dual_manipulation_shared::ik_service::Request req);

  /**
   * @brief handler function for grasping an object
   *
   * @param req
   *   the same req from the @e ik_service
   * @return void
   */
  void grasp(cwru_davinci_dual_manipulation_shared::ik_service::Request req);

  /**
   * @brief handler function for ungrasping an object
   *
   * @param req
   *   the same req from the @e ik_service
   * @return void
   */
  void ungrasp(cwru_davinci_dual_manipulation_shared::ik_service::Request req);

  /**
   * @brief clear all current busy flags
   *
   */
  inline void free_all()
  {
    map_mutex_.lock();
    for (auto &item:busy)
      for (auto &item2:item.second)
        item2.second = false;
    map_mutex_.unlock();
    reset();
  }

  /**
   * @brief resets all robot states and movePlans
   */
  void reset();

  /**
   * @brief function to check whether a capability is busy, and to lock it in case it is
   *
   * @param ee_name
   *    end-effector name
   * @param capability
   *    capability to check
   */
  bool isFreeMakeBusy(std::string ee_name, std::string capability);

  /**
   * @brief add a target to the internal targets list
   *
   * @param req the same req from the @e ik_service
   */
  void addTarget(const dual_manipulation_shared::ik_service::Request &req);

  /**
   * @brief create instances of the various capabilities which will be used inside ik_control
   */
  void instantiateCapabilities();
};

}
}
#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H
