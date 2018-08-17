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
 * Description: A class to manage the group structure used in cwru_davinci_dualmanipulation_ik_control.
 *
 * This class allows the conversion between names used in cwru_davinci_ik_control and
 * group names defined in the robot SRDF file using the parameters loaded on the server.
 * It also provides some useful functionalities to find the right group to use in some circumstances.
 */

#ifndef CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_GROUP_STRUCTURE_MANAGER_H
#define CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_GROUP_STRUCTURE_MANAGER_H

#include <map>
#include <XmlRpcValue.h>

class DavinciGroupStructureManager
{
public:
  DavinciGroupStructureManager(XmlRpc::XmlRpcValue& params);
  ~DavinciGroupStructureManager() {}

  /**
   * @brief Return the SRDF group name associated to an ik_control group name
   *
   * @param ik_control_group
   * @param group_name
   *
   * @return false if the group doesn't exist, true otherwise
   */
  bool getGroupInSRDF(const std::string& ik_control_group, std::string& group_name) const;

  /**
   * @brief Return a map of ik_control > SRDF group names
   * @return
   */
  const std::map<std::string, std::string>& getGroupMap() const;

  /**
   * @brief Return a vector containing known end-effectors (chain-type groups)
   * @return
   */
  const std::vector<std::string>& getChains() const;

  /**
   * @brief True if the ik_control group passed as input is a chain
   *
   * @param group
   *
   * @return
   */
  bool isChain(const std::string& group) const;

  /**
   * @brief True if the ik_control group passed as input is a tree
   *
   * @param group
   *
   * @return
   */
  bool isTree(const std::string& group) const;

  /**
   * @brief Return a vector containing all chains in a tree (empty if the tree is not present)

   * @param group

   * @return
   */
  const std::vector<std::string>& getTreeComposition(const std::string& group) const;

  /**
   * @brief Return a vector containing all trees which contain the chain
   * @param group
   * @return
   */
  std::vector<std::string> getTreeWithChain(const std::string& group) const;

  /**
   * @brief Find the smallest group which contains the listed end-effectors
   * @param ee_list the list of end-effectors to look for
   * @return The name of the smallest ik_control group containing all end-effectors
   */
  std::string findGroupName(const std::vector<std::string>& ee_list) const;

private:
  std::map<std::string, std::string> group_map_;
  std::vector<std::string> chain_names_list_;
  std::vector<std::string> tree_names_list_;
  std::map<std::string,std::vector<std::string>> tree_composition_;

  std::vector<std::string> empty_vector;

private:
  /**
   * @brief utility function to parse parameters and obtain the needed structure
   *
   * @param params parameters got from the parameter server
   *
   * @return void
   */
  void parseParameters(XmlRpc::XmlRpcValue& params);

};
#endif //CWRU_DAVINCI_DUAL_MANIPULATION_IK_CONTROL_DAVINCI_GROUP_STRUCTURE_MANAGER_H
