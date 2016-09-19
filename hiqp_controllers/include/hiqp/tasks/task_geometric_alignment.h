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
 * \file   task_geometric_alignment.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_GEOMETRIC_ALIGNMENT_H
#define HIQP_TASK_GEOMETRIC_ALIGNMENT_H

// STL Includes
#include <string>
#include <vector>

// HiQP Includes
#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_function.h>





namespace hiqp
{
namespace tasks
{

/*!
 * \class TaskGeometricAlignment
 * \brief 
 */  
template<typename PrimitiveA, typename PrimitiveB>
class TaskGeometricAlignment : public TaskFunction
{
public:
     
  TaskGeometricAlignment();

  ~TaskGeometricAlignment() noexcept;

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
  TaskGeometricAlignment(const TaskGeometricAlignment& other) = delete;
  TaskGeometricAlignment(TaskGeometricAlignment&& other) = delete;
  TaskGeometricAlignment& operator=(const TaskGeometricAlignment& other) = delete;
  TaskGeometricAlignment& operator=(TaskGeometricAlignment&& other) noexcept = delete;

  int align(PrimitiveA* first, PrimitiveB* second);

  // v1 must relate to primitive_a_, and v2 to primitive_b_ !
  int alignVectors(const KDL::Vector& v1, const KDL::Vector v2);

  PrimitiveA*              primitive_a_;
  KDL::Frame               pose_a_;
  KDL::Jacobian            jacobian_a_;

  PrimitiveB*              primitive_b_;
  KDL::Frame               pose_b_;
  KDL::Jacobian            jacobian_b_;

  double                   delta_; // the angular error margin

}; // class TaskGeometricAlignment

} // namespace tasks

} // namespace hiqp

#include <hiqp/tasks/task_geometric_alignment__impl.h>

#endif // include guard