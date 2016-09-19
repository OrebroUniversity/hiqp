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
 * \file   task_geometric_alignment__impl.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_GEOMETRIC_ALIGNMENT__IMPL_H
#define HIQP_TASK_GEOMETRIC_ALIGNMENT__IMPL_H

// STL Includes
#include <sstream>
#include <iterator>

// Orocos KDL Includes
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>





namespace hiqp
{
namespace tasks
{





template<typename PrimitiveA, typename PrimitiveB>
TaskGeometricAlignment<PrimitiveA, PrimitiveB>::TaskGeometricAlignment()
{}





template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::init
(
  const HiQPTimePoint& sampling_time,
    const std::vector<std::string>& parameters,
    const KDL::Tree& kdl_tree, 
    unsigned int num_controls
)
{

  if (parameters.size() != 4)
    return -1;

  std::stringstream ss(parameters.at(2));
  std::vector<std::string> args(
    std::istream_iterator<std::string>{ss},
    std::istream_iterator<std::string>{});

  if (args.size() != 3)
    return -2;

  e_.resize(1);
  J_.resize(1, num_controls);
  e_dot_star_.resize(1);
  performance_measures_.resize(1);

  primitive_a_ = geometric_primitive_map_->getGeometricPrimitive<PrimitiveA>(args.at(0));
  primitive_b_ = geometric_primitive_map_->getGeometricPrimitive<PrimitiveB>(args.at(2));

  geometric_primitive_map_->addDependencyToPrimitive(args.at(0), this->getTaskName());
  geometric_primitive_map_->addDependencyToPrimitive(args.at(2), this->getTaskName());

  int sign = 0;

  if (args.at(1).compare("<") == 0 || 
    args.at(1).compare("<=") == 0)
  {
    sign = -1;
  }
  else if (args.at(1).compare("=") == 0 || 
           args.at(1).compare("==") == 0)
  {
    sign = 0;
  }
  else if (args.at(1).compare(">") == 0 || 
         args.at(1).compare(">=") == 0)
  {
    sign = 1;
  }
  else
  {
    return -3;
  }

  delta_ = std::stod( parameters.at(3) );

  task_types_.insert(task_types_.begin(), 1, sign);

  return 0;
}





template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::monitor()
{
  performance_measures_.at(0) = e_(0);
  
  return 0;
}





template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::apply
(
  const HiQPTimePoint& sampling_time,
  const KDL::Tree& kdl_tree, 
  const KDL::JntArrayVel& kdl_joint_pos_vel
)
{

  KDL::TreeFkSolverPos_recursive fk_solver_pos(kdl_tree);
  KDL::TreeJntToJacSolver fk_solver_jac(kdl_tree);

  int retval = 0;


  // Get pose_a_

  retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
                                 pose_a_,
                                 primitive_a_->getFrameId());

  if (retval != 0)
  {
    std::cerr << "In TaskGeometricAlignment::apply : Can't solve position "
      << "of link '" << primitive_a_->getFrameId() << "'" << " in the "
      << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
      << "error code '" << retval << "'\n";
    return -1;
  }


  // Get pose_b_

  retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
                                 pose_b_,
                                 primitive_b_->getFrameId());

  if (retval != 0)
  {
    std::cerr << "In TaskGeometricAlignment::apply : Can't solve position "
      << "of link '" << primitive_b_->getFrameId() << "'" << " in the "
      << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
      << "error code '" << retval << "'\n";
    return -2;
  }


  // Get jacobian_a_

  jacobian_a_.resize(kdl_joint_pos_vel.q.rows());

  retval = fk_solver_jac.JntToJac(kdl_joint_pos_vel.q,
                                jacobian_a_,
                                primitive_a_->getFrameId());

  if (retval != 0)
  {
    std::cerr << "In TaskGeometricAlignment::apply : Can't solve jacobian "
      << "of link '" << primitive_a_->getFrameId() << "'" << " in the "
      << "KDL tree! KDL::TreeJntToJacSolver return error code "
      << "'" << retval << "'\n";
    return -3;
  }


  // Get jacobian_b_

  jacobian_b_.resize(kdl_joint_pos_vel.q.rows());

  retval = fk_solver_jac.JntToJac(kdl_joint_pos_vel.q,
                                jacobian_b_,
                                primitive_b_->getFrameId());

  if (retval != 0)
  {
    std::cerr << "In TaskGeometricAlignment::apply : Can't solve jacobian "
      << "of link '" << primitive_b_->getFrameId() << "'" << " in the "
      << "KDL tree! KDL::TreeJntToJacSolver return error code "
      << "'" << retval << "'\n";
    return -4;
  }

  return align(primitive_a_, primitive_b_);

}





template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricAlignment<PrimitiveA, PrimitiveB>::alignVectors
(
  const KDL::Vector& v1, 
  const KDL::Vector v2
)
{
  double d = KDL::dot(v1, v2);

  e_(0) = d - std::cos(delta_);

  //std::cout << "e = " << e_(0) << "\n";

  KDL::Vector v = v1 * v2;    // v = v1 x v2

  for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
  {
    KDL::Vector Ja = jacobian_a_.getColumn(q_nr).rot;
    KDL::Vector Jb = jacobian_b_.getColumn(q_nr).rot;

    J_(0, q_nr) = KDL::dot( v, (Ja - Jb) );
  }

  return 0;
}





} // namespace tasks

} // namespace hiqp

#endif // include guard