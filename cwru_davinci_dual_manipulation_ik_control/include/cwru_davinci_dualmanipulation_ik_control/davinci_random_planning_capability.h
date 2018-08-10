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

#include <cwru_davinci_dual_manipulation_ik_control/davinci_generic_planning_capability.h>
#include <atomic>
#include <moveit/robot_model_loader/robot_model_loader.h>

namespace cwru_davinci_dual_manipulation
{
namespace cwru_davinci_ik_control
{

class DavinciRandomPlanningCapability : public DavinciGenericPlanningCapability
{
public:
  DavinciRandomPlanningCapability(shared_ik_memory& sikm_, const ros::NodeHandle& node_ = ros::NodeHandle());
  virtual ~DavinciRandomPlanningCapability();
  virtual bool isComplete();
  virtual void performRequest(dual_manipulation_shared::ik_serviceRequest req);
  virtual bool getResults(dual_manipulation_shared::ik_response& res);
  virtual bool canRun();
  virtual bool canPerformCapability(const ik_control_capabilities& ik_capability) const;
  virtual void reset();

  /**
   * @brief add a target to the internal targets list
   *
   * @param req the same req from the @e ik_service
   */
  void add_target(const dual_manipulation_shared::ik_service::Request& req);

private:
  
};
}
}
#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_RANDOM_PLANNING_CAPABILITY_H
