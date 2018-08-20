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
  ~DavinciIkControl(){};

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
  std::map<ik_control_capabilities,ros::Publisher> hand_pub_;

  // utility variables
  std::vector<std::unique_ptr<std::thread>> used_threads_;
  std::map<ik_control_capability_types,std::map<std::string,bool>> busy;
  const ik_control_capability capabilities_;
  std::mutex map_mutex_; // busy
  std::mutex ikCheck_mutex_;
  std::string joint_states_;
  std::string robot_description_;
  std::string full_robot_group_;

};

}
}
#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H
