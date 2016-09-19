// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

/*
 * \file   task_jnt_limits.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/tasks/task_jnt_limits.h>

#include <hiqp/hiqp_utils.h>

#include <iostream>





namespace hiqp
{
namespace tasks
{





int TaskJntLimits::init
(
  const HiQPTimePoint& sampling_time,
  const std::vector<std::string>& parameters,
  const KDL::Tree& kdl_tree, 
  unsigned int num_controls
)
{
  int size = parameters.size();
  if (size != 4)
  {
    printHiqpWarning("TaskJntLimits requires 4 parameter, got " 
      + std::to_string(size) + "! Initialization failed!");
    return -1;
  }

  e_.resize(4);
  J_.resize(4, num_controls);
  e_dot_star_.resize(4);
  performance_measures_.resize(0);

  task_types_.resize(4);
  task_types_.at(0) = 1; // > -dq_max
  task_types_.at(1) = -1; // < dq_max
  task_types_.at(2) = 1; // > lower bound
  task_types_.at(3) = -1; // < upper bound

  link_frame_name_ = parameters.at(0);
  link_frame_q_nr_ = kdl_getQNrFromLinkName(kdl_tree, link_frame_name_);
  dq_max_ = std::stod( parameters.at(1) );
  jnt_lower_bound_ = std::stod( parameters.at(2) );
  jnt_upper_bound_ = std::stod( parameters.at(3) );

  for (int i=0; i<4; ++i) 
  {
    for (int j=0; j<num_controls; ++j) 
    {
      J_(i, j) = (j == link_frame_q_nr_ ? 1 : 0);
    }
  }
  
  return 0;
}





int TaskJntLimits::apply
(
  const HiQPTimePoint& sampling_time,
  const KDL::Tree& kdl_tree, 
  const KDL::JntArrayVel& kdl_joint_pos_vel
)
{
  double q = kdl_joint_pos_vel.q(link_frame_q_nr_);
  
  e_(0) = q;
  e_(1) = q;
  e_(2) = q - jnt_lower_bound_;
  e_(3) = q - jnt_upper_bound_;

  return 0;
}





int TaskJntLimits::monitor()
{
  return 0;
}





} // namespace tasks

} // namespace hiqp