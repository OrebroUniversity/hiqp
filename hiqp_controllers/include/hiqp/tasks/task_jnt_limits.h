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
 * \file   task_jnt_limits.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_JNT_LIMITS_H
#define HIQP_TASK_JNT_LIMITS_H

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
 * \class TaskJntLimits
 * \brief Task achieving velocity and position limitations of a specific joint.
 */  
class TaskJntLimits : public TaskFunction
{
public:

  TaskJntLimits() {}

  ~TaskJntLimits() noexcept {}

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
  TaskJntLimits(const TaskJntLimits& other) = delete;
  TaskJntLimits(TaskJntLimits&& other) = delete;
  TaskJntLimits& operator=(const TaskJntLimits& other) = delete;
  TaskJntLimits& operator=(TaskJntLimits&& other) noexcept = delete;

  std::string              link_frame_name_;

  std::size_t              link_frame_q_nr_;

  double                   dq_max_;

  double                   jnt_upper_bound_;

  double                   jnt_lower_bound_;

}; // class TaskJntLimits

} // namespace tasks

} // namespace hiqp

#endif // include guard