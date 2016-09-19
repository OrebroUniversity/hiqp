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
 * \file   task_jnt_config.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_JNT_CONFIG_H
#define HIQP_TASK_JNT_CONFIG_H

// STL Includes
#include <string>

// HiQP Includes
#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_function.h>





namespace hiqp
{
namespace tasks
{

/*!
 * \class TaskJntConfig
 * \brief Represents a task that sets a specific joint configuration
 *
 *  This task does not leave any redundancy available to other tasks.
 */  
class TaskJntConfig : public TaskFunction
{
public:

  TaskJntConfig() {}

  ~TaskJntConfig() noexcept {}

  int init
  (
    const HiQPTimePoint& sampling_time,
    const std::vector<std::string>& parameters,
    const KDL::Tree& kdl_tree, 
    unsigned int num_controls
  );

  int apply
  (
    const HiQPTimePoint& sampling_time,
    const KDL::Tree& kdl_tree, 
    const KDL::JntArrayVel& kdl_joint_pos_vel
  );

  int monitor();



private:
  // No copying of this class is allowed !
  TaskJntConfig(const TaskJntConfig& other) = delete;
  TaskJntConfig(TaskJntConfig&& other) = delete;
  TaskJntConfig& operator=(const TaskJntConfig& other) = delete;
  TaskJntConfig& operator=(TaskJntConfig&& other) noexcept = delete;

  std::string              link_name_;
  
  int                      joint_q_nr_;

  double                   desired_configuration_;

}; // class TaskJntConfig

} // namespace tasks

} // namespace hiqp

#endif // include guard