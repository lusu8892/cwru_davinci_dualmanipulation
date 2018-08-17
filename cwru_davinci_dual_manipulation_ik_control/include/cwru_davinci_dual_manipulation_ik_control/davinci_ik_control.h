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
 * Description: A class to manage a single object in an atomic manner,
 * making sure that it gets assigned a desired value (when this object goes out of scope,
 * it makes the assignment using the appropriate LOCK-able variable)
 */


#include <dual_manipulation_shared/ik_service.h>
#include <dual_manipulation_shared/scene_object_service.h>
#include "ik_check_capability/ik_check_capability.h"
#include <thread>
#include <XmlRpcValue.h>
#include <mutex>
#include <std_msgs/String.h>

// capabilities definition
#include <dual_manipulation_shared/ik_control_capabilities.h>
#include <cwru_davinci_dual_manipulation_ik_control/random_planning_capability.h>
#include <cwru_davinci_dual_manipulation_ik_control/trajectory_execution_capability.h>
#include <cwru_davinci_dual_manipulation_ik_control/grasping_capability.h>


#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H

#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_IK_CONTROL_H
