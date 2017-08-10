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

#ifndef HIQP_TDEF_GEOMETRIC_PROJECTION__IMPL_H
#define HIQP_TDEF_GEOMETRIC_PROJECTION__IMPL_H

#include <iterator>
#include <sstream>

#include <hiqp/utilities.h>

namespace hiqp {
namespace tasks {

template <typename PrimitiveA, typename PrimitiveB>
TDefGeometricProjection<PrimitiveA, PrimitiveB>::TDefGeometricProjection(
    std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
    std::shared_ptr<Visualizer> visualizer)
    : TaskDefinition(geom_prim_map, visualizer) {}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjection<PrimitiveA, PrimitiveB>::init(
    const std::vector<std::string>& parameters, RobotStatePtr robot_state) {
  int parameters_size = parameters.size();
  if (parameters_size != 4) {
    printHiqpWarning(
        "'" + getTaskName() + "': TDefGeomProj takes 4 parameters, got " +
        std::to_string(parameters_size) + "! The task was not added!");
    return -1;
  }

  // Used for debugging
  // if (parameters.at(2).compare("box") == 0) {
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_center", "point",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0});
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_proj", "point",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0});
  //   getGeometricPrimitiveMap()->addGeometricPrimitive("box_line", "line",
  //   "world", true, {0, 0, 1, 1}, {0, 0, 0, 0, 0, 0});
  // }

  std::stringstream ss(parameters.at(3));
  std::vector<std::string> args(std::istream_iterator<std::string>{ss},
                                std::istream_iterator<std::string>{});

  if (args.size() != 3) {
    printHiqpWarning("'" + getTaskName() +
                     "': TDefGeomProj's parameter nr.4 needs whitespace "
                     "separation! The task was not added!");
    return -2;
  }

  unsigned int n_joints = robot_state->getNumJoints();
  e_.resize(1);
  J_.resize(1, n_joints);
  e_dot_.resize(1);
  J_dot_.resize(1, n_joints);

  performance_measures_.resize(0);

  fk_solver_pos_ =
      std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
  fk_solver_jac_ =
      std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);

  std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();

  primitive_a_ = gpm->getGeometricPrimitive<PrimitiveA>(args.at(0));
  if (primitive_a_ == nullptr) {
    printHiqpWarning(
        "In TDefGeometricProjection::init(), couldn't find primitive with name "
        "'" +
        args.at(0) + "'. Unable to create task!");
    return -3;
  }

  primitive_b_ = gpm->getGeometricPrimitive<PrimitiveB>(args.at(2));
  if (primitive_b_ == nullptr) {
    printHiqpWarning(
        "In TDefGeometricProjection::init(), couldn't find primitive with name "
        "'" +
        args.at(2) + "'. Unable to create task!");
    return -3;
  }

  gpm->addDependencyToPrimitive(args.at(0), this->getTaskName());
  gpm->addDependencyToPrimitive(args.at(2), this->getTaskName());

  int sign = 0;

  if (args.at(1).compare("<") == 0 || args.at(1).compare("<=") == 0) {
    sign = -1;
  } else if (args.at(1).compare("=") == 0 || args.at(1).compare("==") == 0) {
    sign = 0;
  } else if (args.at(1).compare(">") == 0 || args.at(1).compare(">=") == 0) {
    sign = 1;
  } else {
    return -4;
  }

  task_types_.clear();
  task_types_.insert(task_types_.begin(), 1, sign);

  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjection<PrimitiveA, PrimitiveB>::update(
    RobotStatePtr robot_state) {
  int retval = 0;

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_a_,
                                     primitive_a_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve position "
              << "of link '" << primitive_a_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
              << "error code '" << retval << "'\n";
    return -1;
  }

  retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_b_,
                                     primitive_b_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve position "
              << "of link '" << primitive_b_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
              << "error code '" << retval << "'\n";
    return -2;
  }

  jacobian_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
                                    jacobian_a_, primitive_a_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian "
              << "of link '" << primitive_a_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeJntToJacSolver return error code "
              << "'" << retval << "'\n";
    return -3;
  }

  jacobian_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = fk_solver_jac_->JntToJac(robot_state->kdl_jnt_array_vel_.q,
                                    jacobian_b_, primitive_b_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian "
              << "of link '" << primitive_b_->getFrameId() << "'"
              << " in the "
              << "KDL tree! KDL::TreeJntToJacSolver return error code "
              << "'" << retval << "'\n";
    return -4;
  }


  jacobian_dot_a_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = treeJntToJacDot(robot_state->kdl_tree_, jacobian_a_, robot_state->kdl_jnt_array_vel_,
  			   jacobian_dot_a_, primitive_a_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian derivative "
              << "of link '" << primitive_a_->getFrameId() << "'"
              << " in the "
              << "KDL tree! treeJntToJacDot return error code "
              << "'" << retval << "'\n";
    return -5;
  }

    jacobian_dot_b_.resize(robot_state->kdl_jnt_array_vel_.q.rows());
  retval = treeJntToJacDot(robot_state->kdl_tree_, jacobian_b_, robot_state->kdl_jnt_array_vel_,
  			   jacobian_dot_b_, primitive_b_->getFrameId());
  if (retval != 0) {
    std::cerr << "In TDefGeometricProjection::update : Can't solve jacobian derivative "
              << "of link '" << primitive_b_->getFrameId() << "'"
              << " in the "
              << "KDL tree! treeJntToJacDot return error code "
              << "'" << retval << "'\n";
    return -6;
  }
  
  project(primitive_a_, primitive_b_,robot_state->kdl_jnt_array_vel_);
  maskJacobian(robot_state);
  maskJacobianDerivative(robot_state);  
  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
int TDefGeometricProjection<PrimitiveA, PrimitiveB>::monitor() {
  return 0;
}

template <typename PrimitiveA, typename PrimitiveB>
KDL::Vector TDefGeometricProjection<PrimitiveA, PrimitiveB>::
    getRelativeVelocityJacobian(const KDL::Vector& p1,
                                    const KDL::Vector& p2, int q_nr) {
  KDL::Twist Ja = jacobian_a_.getColumn(q_nr);
  KDL::Twist Jb = jacobian_b_.getColumn(q_nr);
  KDL::Vector Jp1 = Ja.rot * p1;
  KDL::Vector Jp2 = Jb.rot * p2;

  return (Jb.vel + Jp2 - (Ja.vel + Jp1));
}

 template <typename PrimitiveA, typename PrimitiveB>
   KDL::Jacobian TDefGeometricProjection<PrimitiveA, PrimitiveB>::
   getRelativeVelocityJacobianDerivative(const KDL::Vector& p1,
					 const KDL::Vector& p2,
					 const KDL::JntArrayVel& qqdot) {
   const KDL::JntArray& qdot_in = qqdot.qdot;
   unsigned int n=jacobian_dot_a_.columns();
   assert(qdot_in.rows() == n);
   
   KDL::Jacobian jac_dot_rel(n);
   KDL::Jacobian jac_dot_a(n);
   KDL::Jacobian jac_dot_b(n);
   KDL::Vector dv_a;
   KDL::Vector dv_b;
   SetToZero(dv_a);
   SetToZero(dv_b);
   //compute relative velocities between end-effectors and link origins - should do this via jacobians rather than a separate loop ...
   for (unsigned int i=0; i<n;i++){
     dv_a+=(qdot_in(i) * jacobian_a_.getColumn(i).rot) * p1;
     dv_b+=(qdot_in(i) * jacobian_b_.getColumn(i).rot) * p2;
   }

   
   for (unsigned int i=0; i<n;i++){
     KDL::Twist col_a=jacobian_dot_a_.getColumn(i);
     KDL::Twist col_b=jacobian_dot_b_.getColumn(i);

     col_a.vel+=col_a.rot * p1 + jacobian_a_.getColumn(i).rot * dv_a;
     col_b.vel+=col_b.rot * p2 + jacobian_b_.getColumn(i).rot * dv_b;

     jac_dot_a.setColumn(i,col_a);
     jac_dot_b.setColumn(i,col_b);
     jac_dot_rel.setColumn(i,col_b - col_a);
   }
   //DEBUG ========================================================
   /* std::cerr<<"q_in: "<<qqdot.q.data.transpose()<<std::endl; */
   /* std::cerr<<"dq_in: "<<qdot_in.data.transpose()<<std::endl<<std::endl; */

   /* std::cerr<<"p1 "<<p1.x()<<" "<<p1.y()<<" "<<p1.z()<<std::endl; */
   /* std::cerr<<"p2 "<<p2.x()<<" "<<p2.y()<<" "<<p2.z()<<std::endl<<std::endl;    */

   /* std::cerr<<"jacobian_dot_a_: "<<std::endl<<jacobian_dot_a_.data<<std::endl; */
   /* std::cerr<<"jacobian_dot_b_: "<<std::endl<<jacobian_dot_b_.data<<std::endl<<std::endl; */

   /* std::cerr<<"jac_dot_a: "<<std::endl<<jac_dot_a.data<<std::endl; */
   /* std::cerr<<"jac_dot_b: "<<std::endl<<jac_dot_b.data<<std::endl;  */     
   //DEBUG END ====================================================
   
   return jac_dot_rel;
 }

 
template <typename PrimitiveA, typename PrimitiveB>
void TDefGeometricProjection<PrimitiveA, PrimitiveB>::maskJacobian(
    RobotStatePtr robot_state) {
  for (unsigned int c = 0; c < robot_state->getNumJoints(); ++c) {
    if (!robot_state->isQNrWritable(c)){
      J_.col(c).setZero();
    }
  }
 }
  
template <typename PrimitiveA, typename PrimitiveB>
void TDefGeometricProjection<PrimitiveA, PrimitiveB>::maskJacobianDerivative(
    RobotStatePtr robot_state) {
  for (unsigned int c = 0; c < robot_state->getNumJoints(); ++c) {
    if (!robot_state->isQNrWritable(c)){
            J_dot_.col(c).setZero();
    }
  }
 }

}  // namespace tasks

}  // namespace hiqp

#endif  // include guard
