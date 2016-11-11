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
 * \file   task_geometric_projection.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_GEOMETRIC_PROJECTION_H
#define HIQP_TASK_GEOMETRIC_PROJECTION_H

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
 * \class TaskGeometricProjection
 * \brief 
 */  
template<typename PrimitiveA, typename PrimitiveB>
class TaskGeometricProjection : public TaskFunction
{
public:

  TaskGeometricProjection();

  ~TaskGeometricProjection() noexcept {}

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

  int project(std::shared_ptr<PrimitiveA> first, std::shared_ptr<PrimitiveB> second);



private:
  // No copying of this class is allowed !
  TaskGeometricProjection(const TaskGeometricProjection& other) = delete;
  TaskGeometricProjection(TaskGeometricProjection&& other) = delete;
  TaskGeometricProjection& operator=(const TaskGeometricProjection& other) = delete;
  TaskGeometricProjection& operator=(TaskGeometricProjection&& other) noexcept = delete;

  

  /*! \brief Computes column number q_nr of the resulting jacobian for the 
   *         vector (p2-p1), NOTE! p1 must be related to pose_a_ and p2 to 
   *         pose_b_ !
   */
  KDL::Vector getVelocityJacobianForTwoPoints
  (
    const KDL::Vector& p1, 
    const KDL::Vector& p2,
    int q_nr
  );

  std::shared_ptr<PrimitiveA>  primitive_a_;
  KDL::Frame                   pose_a_;
  KDL::Jacobian                jacobian_a_;

  std::shared_ptr<PrimitiveB>  primitive_b_;
  KDL::Frame                   pose_b_;
  KDL::Jacobian                jacobian_b_;

}; // class TaskGeometricProjection

} // namespace tasks

} // namespace hiqp

#include <hiqp/tasks/task_geometric_projection__impl.h>

#endif // include guard