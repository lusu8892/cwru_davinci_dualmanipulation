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

#include <cwru_davinci_dual_manipulation_ik_control/davinci_shared_ik_memory.h>

#include <cwru_davinci_dual_manipulation_shared/parsing_utils.h>

using namespace cwru_davinci_dual_manipulation::cwru_davinci_ik_control;

DavinciSharedIKMemory::DavinciSharedIKMemory(XmlRpc::XmlRpcValue& params, const ros::NodeHandle& nh)
{

}

void DavinciSharedIKMemory::reset()
{
  void;
}

bool DavinciSharedIKMemory::swapTrajectory(const std::string& group, moveit_msgs::RobotTrajectory& traj)
{
  void;
}

bool DavinciSharedIKMemory::setPendingTrajectoryExecution()
{
  void;
}

bool DavinciSharedIKMemory::setNextTrajectoryRelativeEndTime(const ros::Duration& dt)
{
  void;
}

bool DavinciSharedIKMemory::getNextTrajectoryEndTime(ros::Time& end_t)
{
  void;
}

moveit::core::RobotState DavinciSharedIKMemory::getPlanningRobotState()
{
  void;
}

bool DavinciSharedIKMemory::resetPlanningRobotState(const std::string& group)
{
  void;
}

bool DavinciSharedIKMemory::resetPlanningRobotState(const std::string& group, const moveit_msgs::RobotTrajectory& traj)
{}

void DavinciSharedIKMemory::parseParameters(XmlRpc::XmlRpcValue& params)
{}