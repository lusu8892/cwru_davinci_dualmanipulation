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

#include <cwru_davinci_dualmanipulation_ik_control/group_structure_manager.h>
#include <cwru_davinci_dualmanipulation_shared/parsing_utils.h>
#include <assert.h>
#include <iostream>
#include <string>
#include <algorithm>

#define RED_INTENSE "\033[0;91m"
#define NO_COLOR "\033[0m"
#define CLASS_NAMESPACE "GroupStructureManager::"

DavinciGroupStructureManager::DavinciGroupStructureManager(XmlRpc::XmlRpcValue& params)
{
  parseParameters(params);
}

void DavinciGroupStructureManager::parseParameters(XmlRpc::XmlRpcValue& params)
{
  assert(params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  bool correct(true);

  parseSingleParameter(params,chain_names_list_,"chain_group_names",1);
  parseSingleParameter(params,tree_names_list_,"tree_group_names",1);

  // list of chains composing each tree
  if(params.hasMember("tree_composition"))
  {
    std::map<std::string, std::vector<std::string>> tree_composition_tmp;

    for(auto tree:tree_names_list_)
    {
      parseSingleParameter(params["tree_composition"],tree_composition_tmp[tree], tree);
      if(tree_composition_tmp.at(tree).empty())
      {
        tree_composition_tmp.erase(tree);
      }
      else
      {
        for(auto chain:tree_composition_tmp(tree))
        {
          if(std::find(chain_names_list_.begin(),chain_names_list_.end(),chain) == chain_names_list_.end())
          {
            std::cout << RED_INTENSE << CLASS_NAMESPACE << __func__
                      << " : bad chain name \'" << chain << "\' specified in composition for tree '"
                      << tree << "': check the yaml configuration." << NO_COLOR << std::endl;

            correct = false;
          }
        }
      }
    }
    if(!tree_composition_tmp.empty())
    {
      tree_composition_.swap(tree_composition_tmp);
      tree_composition_tmp.clear();
    }

    std::vector<std::string> tree_names_tmp;
    tree_names_tmp.swap(tree_names_list_);

    for(auto tree:tree_names_tmp)
    {
      if(!tree_composition_.count(tree) || tree_composition_.at(tree).size() == 0)
      {
        std::cout << RED_INTENSE << CLASS_NAMESPACE << __func__
                  << " : No composition is specified for tree '"
                  << tree << "': check the yaml configuration."
                  << NO_COLOR << std::endl;

        correct = false;
      }
      else
        tree_names_list_.push_back(tree);

      std::map<std::string, std::string> map_tmp, map_tmp_tree;
      

    }


  }

}
