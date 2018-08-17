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

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_SCENE_OBJECT_MANAGER_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_SCENE_OBJECT_MANAGER_H

#include <ros/ros.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include <dual_manipulation_shared/databasemapper.h>
#include <cwru_davinci_dualmanipulation_ik_control/davinci_group_structure_manager.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <mutex>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace cwru_davinci_dual_manipulation
{
namespace cwru_davinci_ik_control
{

static const std::string ADD_OBJECT("add");
static const std::string REMOVE_OBJECT("remove");
static const std::string ATTACH_OBJECT("attach");
static const std::string DETACH_OBJECT("detach");
static const std::string REMOVE_ALL_OBJECTS("remove_all");



}
}

#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_SCENE_OBJECT_MANAGER_H
