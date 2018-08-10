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
 * Description:
 */

#ifndef SHARED_DAVINCI_IK_CONTROL_CAPABILITIES_H
#define SHARED_DAVINCI_IK_CONTROL_CAPABILITIES_H

#include <map>
#include <string>

#define SET_TARGET_CAPABILITY "set_target"
#define SET_HOME_TARGET_CAPABILITY "set_home_target"
#define IK_CHECK_CAPABILITY "ik_check"
#define PLAN_CAPABILITY "plan"
#define PLAN_NO_COLLISION_CAPABILITY "plan_no_collision"
#define PLAN_BEST_EFFORT_CAPABILITY "plan_best_effort"
#define PLAN_CLOSE_BEST_EFFORT_CAPABILITY "plan_close_best_effort"
#define MOVE_CAPABILITY "execute"
#define GRASP_CAPABILITY "grasp"
#define UNGRASP_CAPABILITY "ungrasp"
#define HOME_CAPABILITY "home"

#define SET_TARGET_MSG "set_target_done"
#define SET_HOME_TARGET_MSG "set_target_done"
#define IK_CHECK_MSG "check_done"
#define PLAN_MSG "planning_done"
#define PLAN_NO_COLLISION_MSG "planning_done"
#define PLAN_BEST_EFFORT_MSG "planning_done"
#define PLAN_CLOSE_BEST_EFFORT_MSG "planning_done"
#define MOVE_MSG "action_done"
#define GRASP_MSG "grasp_done"
#define UNGRASP_MSG "ungrasp_done"
#define HOME_MSG "action_done"

enum class IKControlCapabilityTypes
{
  SET_TARGET,
  IK_CHECK,
  PLAN,
  MOVE,
  GRASP
};

enum class IKControlCapabilities
{
  SET_TARGET,
  SET_HOME_TARGET,
  IK_CHECK,
  PLAN,
  PLAN_NO_COLLISION,
  PLAN_BEST_EFFORT,
  PLAN_CLOSE_BEST_EFFORT,
  MOVE,
  GRASP,
  UNGRASP,
  HOME
};

class DavinciIKControlCapability
{
public:
  DavinciIKControlCapability()
  {
    name[IKControlCapabilities::SET_TARGET] = SET_TARGET_CAPABILITY;
    name[IKControlCapabilities::SET_HOME_TARGET] = SET_HOME_TARGET_CAPABILITY;
    name[IKControlCapabilities::IK_CHECK] = IK_CHECK_CAPABILITY;
    name[IKControlCapabilities::PLAN] = PLAN_CAPABILITY;
    name[IKControlCapabilities::PLAN_NO_COLLISION] = PLAN_NO_COLLISION_CAPABILITY;
    name[IKControlCapabilities::PLAN_BEST_EFFORT] = PLAN_BEST_EFFORT_CAPABILITY;
    name[IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT] = PLAN_CLOSE_BEST_EFFORT_CAPABILITY;
    name[IKControlCapabilities::MOVE] = MOVE_CAPABILITY;
    name[IKControlCapabilities::GRASP] = GRASP_CAPABILITY;
    name[IKControlCapabilities::UNGRASP] = UNGRASP_CAPABILITY;
    name[IKControlCapabilities::HOME] = HOME_CAPABILITY;

    from_name[SET_TARGET_CAPABILITY] = IKControlCapabilities::SET_TARGET;
    from_name[SET_HOME_TARGET_CAPABILITY] = IKControlCapabilities::SET_HOME_TARGET;
    from_name[IK_CHECK_CAPABILITY] = IKControlCapabilities::IK_CHECK;
    from_name[PLAN_CAPABILITY] = IKControlCapabilities::PLAN;
    from_name[PLAN_NO_COLLISION_CAPABILITY] = IKControlCapabilities::PLAN_NO_COLLISION;
    from_name[PLAN_BEST_EFFORT_CAPABILITY] = IKControlCapabilities::PLAN_BEST_EFFORT;
    from_name[PLAN_CLOSE_BEST_EFFORT_CAPABILITY] = IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT;
    from_name[MOVE_CAPABILITY] = IKControlCapabilities::MOVE;
    from_name[GRASP_CAPABILITY] = IKControlCapabilities::GRASP;
    from_name[UNGRASP_CAPABILITY] = IKControlCapabilities::UNGRASP;
    from_name[HOME_CAPABILITY] = IKControlCapabilities::HOME;

    msg[IKControlCapabilities::SET_TARGET] = SET_TARGET_MSG;
    msg[IKControlCapabilities::SET_HOME_TARGET] = SET_HOME_TARGET_MSG;
    msg[IKControlCapabilities::IK_CHECK] = IK_CHECK_MSG;
    msg[IKControlCapabilities::PLAN] = PLAN_MSG;
    msg[IKControlCapabilities::PLAN_NO_COLLISION] = PLAN_NO_COLLISION_MSG;
    msg[IKControlCapabilities::PLAN_BEST_EFFORT] = PLAN_BEST_EFFORT_MSG;
    msg[IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT] = PLAN_CLOSE_BEST_EFFORT_MSG;
    msg[IKControlCapabilities::MOVE] = MOVE_MSG;
    msg[IKControlCapabilities::GRASP] = GRASP_MSG;
    msg[IKControlCapabilities::UNGRASP] = UNGRASP_MSG;
    msg[IKControlCapabilities::HOME] = HOME_MSG;

    type[IKControlCapabilities::SET_TARGET] = IKControlCapabilityTypes::SET_TARGET;
    type[IKControlCapabilities::SET_HOME_TARGET] = IKControlCapabilityTypes::SET_TARGET;
    type[IKControlCapabilities::IK_CHECK] = IKControlCapabilityTypes::IK_CHECK;
    type[IKControlCapabilities::PLAN] = IKControlCapabilityTypes::PLAN;
    type[IKControlCapabilities::PLAN_NO_COLLISION] = IKControlCapabilityTypes::PLAN;
    type[IKControlCapabilities::PLAN_BEST_EFFORT] = IKControlCapabilityTypes::PLAN;
    type[IKControlCapabilities::PLAN_CLOSE_BEST_EFFORT] = IKControlCapabilityTypes::PLAN;
    type[IKControlCapabilities::MOVE] = IKControlCapabilityTypes::MOVE;
    type[IKControlCapabilities::GRASP] = IKControlCapabilityTypes::GRASP;
    type[IKControlCapabilities::UNGRASP] = IKControlCapabilityTypes::GRASP;
    type[IKControlCapabilities::HOME] = IKControlCapabilityTypes::MOVE;

    implemented_for_trees[IKControlCapabilityTypes::SET_TARGET] = true;
    implemented_for_trees[IKControlCapabilityTypes::IK_CHECK] = false;
    implemented_for_trees[IKControlCapabilityTypes::PLAN] = true;
    implemented_for_trees[IKControlCapabilityTypes::MOVE] = true;
    implemented_for_trees[IKControlCapabilityTypes::GRASP] = false;

  }

  ~DavinciIKControlCapability(){};

  std::map<IKControlCapabilities,std::string> name;
  std::map<std::string,IKControlCapabilities> from_name;
  std::map<IKControlCapabilities,std::string> msg;

  std::map<IKControlCapabilities,IKControlCapabilityTypes> type;
  std::map<IKControlCapabilityTypes,bool> implemented_for_trees;
};



#endif //SHARED_DAVINCI_IK_CONTROL_CAPABILITIES_H
