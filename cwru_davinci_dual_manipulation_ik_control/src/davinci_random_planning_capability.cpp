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

bool DavinciRandomPlanningCapability::canPerformCapability(const IKControlCapabilities& ik_capability) const
{
  if( (ik_capability == IKControlCapabilities::PLAN)              ||
      (ik_capability == IKControlCapabilities::PLAN_NO_COLLISION) ||
      (ik_capability == IKControlCapabilities::PLAN_BEST_EFFORT)  ||
      (ik_capability == IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT) )
    return true;

  return false;
}

void DavinciRandomPlanningCapability::reset()
{
  std::unique_lock<std::mutex> ul(sikm.m);
  parseParameters(*(sikm.ik_control_parames));

  setParameterDependentVariables();

  busy.store(false);
}
