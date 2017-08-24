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

#ifndef HIQP_TDEF_TRACKING__IMPL_H
#define HIQP_TDEF_TRACKING__IMPL_H

#include <iterator>
#include <sstream>

#include <hiqp/utilities.h>

namespace hiqp {
  namespace tasks {

    template <typename PrimitiveA, typename PrimitiveB>
      TDefTracking<PrimitiveA, PrimitiveB>::TDefTracking(
							 std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
							 std::shared_ptr<Visualizer> visualizer)
      : TaskDefinition(geom_prim_map, visualizer) {
      d_max_=INFINITY;
    }

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefTracking<PrimitiveA, PrimitiveB>::init(const std::vector<std::string>& parameters, RobotStatePtr robot_state) {
      int parameters_size = parameters.size();
      if (parameters_size != 4 && parameters_size != 5) {
	printHiqpWarning("'" + getTaskName() + "': TDefTracking takes 4 or 5 parameters, got " +
			 std::to_string(parameters_size) + "! The task was not added!");
	return -1;
      }

      std::string prim_type1 = parameters.at(1);
      std::string prim_type2 = parameters.at(2);

      std::stringstream ss(parameters.at(3));
      std::vector<std::string> args(std::istream_iterator<std::string>{ss},
				    std::istream_iterator<std::string>{});

      if (args.size() != 3) {
	printHiqpWarning("'" + getTaskName() + "': TDefTracking's parameter nr.4 needs whitespace "
			 "separation! The task was not added!");
	return -2;
      }

      if(parameters_size == 5){
	d_max_= std::stod(parameters.at(4));
	assert(d_max_ > 0.0);
      }

      unsigned int n_task_dimensions = 3;
      if (prim_type1.compare("frame") == 0 && prim_type2.compare("frame") == 0) {
	n_task_dimensions = 6;
      }

      unsigned int n_joints = robot_state->getNumJoints();
      e_ = Eigen::VectorXd::Zero(n_task_dimensions);
      f_ = Eigen::VectorXd::Zero(n_task_dimensions);      
      J_ = Eigen::MatrixXd::Zero(n_task_dimensions, n_joints);
      e_dot_ = Eigen::VectorXd::Zero(n_task_dimensions);
      J_dot_= Eigen::MatrixXd::Zero(n_task_dimensions, n_joints);
      
      fk_solver_pos_ = 	std::make_shared<KDL::TreeFkSolverPos_recursive>(robot_state->kdl_tree_);
      fk_solver_jac_ = 	std::make_shared<KDL::TreeJntToJacSolver>(robot_state->kdl_tree_);

      std::shared_ptr<GeometricPrimitiveMap> gpm = this->getGeometricPrimitiveMap();

      primitive_a_ = gpm->getGeometricPrimitive<PrimitiveA>(args.at(0));
      if (primitive_a_ == nullptr) {
	printHiqpWarning( "In TDefTracking::init(), couldn't find primitive with name "
			  "'" + args.at(0) + "'. Unable to create task!");
	return -3;
      }

      primitive_b_ = gpm->getGeometricPrimitive<PrimitiveB>(args.at(2));
      if (primitive_b_ == nullptr) {
	printHiqpWarning("In TDefTracking::init(), couldn't find primitive with name "
			 "'" + args.at(2) + "'. Unable to create task!");
	return -3;
      }

      gpm->addDependencyToPrimitive(args.at(0), this->getTaskName());
      gpm->addDependencyToPrimitive(args.at(2), this->getTaskName());

      int sign;    
      if (args.at(1).compare("=") == 0 || args.at(1).compare("==") == 0) {
	sign = 0;
      }else {
	printHiqpWarning("'" + getTaskName() + "': TDefTracking needs to be an equality task! The task was not added!");
	return -4;
      }

      task_signs_.clear();
      task_signs_.insert(task_signs_.begin(), n_task_dimensions, sign);
      
      return 0;
    }

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefTracking<PrimitiveA, PrimitiveB>::update(RobotStatePtr robot_state) {
      int retval = 0;

      retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_a_,
					 primitive_a_->getFrameId());
      if (retval != 0) {
	std::cerr << "In TDefTracking::update : Can't solve position "
		  << "of link '" << primitive_a_->getFrameId() << "'"
		  << " in the "
		  << "KDL tree! KDL::TreeFkSolverPos_recursive::JntToCart return "
		  << "error code '" << retval << "'\n";
	return -1;
      }

      retval = fk_solver_pos_->JntToCart(robot_state->kdl_jnt_array_vel_.q, pose_b_,
					 primitive_b_->getFrameId());
      if (retval != 0) {
	std::cerr << "In TDefTracking::update : Can't solve position "
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
	std::cerr << "In TDefTracking::update : Can't solve jacobian "
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
	std::cerr << "In TDefTracking::update : Can't solve jacobian "
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
	std::cerr << "In TDefTracking::update : Can't solve jacobian derivative "
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
	std::cerr << "In TDefTracking::update : Can't solve jacobian derivative "
		  << "of link '" << primitive_b_->getFrameId() << "'"
		  << " in the "
		  << "KDL tree! treeJntToJacDot return error code "
		  << "'" << retval << "'\n";
	return -6;
      }
  
      track(primitive_a_, primitive_b_,robot_state);
      maskJacobian(robot_state);
      maskJacobianDerivative(robot_state);

      //awkward fix to not let the contribution of J_dot get out of hand due to numerical issues with large joint velocities induced by singularities
      double tol=1e5;
      if(fabs((J_dot_*robot_state->kdl_jnt_array_vel_.qdot.data)(0)) > tol){
	J_dot_.setZero();
	e_dot_.setZero();
      }
  
      return 0;
    }

    template <typename PrimitiveA, typename PrimitiveB>
      int TDefTracking<PrimitiveA, PrimitiveB>::alignUnitVectors(const KDL::Vector& v1, const KDL::Vector v2, const RobotStatePtr robot_state, Eigen::VectorXd& e, Eigen::VectorXd& e_dot, Eigen::MatrixXd& J, Eigen::MatrixXd& J_dot) {

      double q_nr=jacobian_a_.columns();
      KDL::Jacobian J_v1, J_v2, J_v1_dot, J_v2_dot;
      J_v1.resize(q_nr);
      J_v2.resize(q_nr);
      J_v1_dot.resize(q_nr);
      J_v2_dot.resize(q_nr);

      changeJacRefPoint(jacobian_a_, v1, J_v1);
      J_v1.data=J_v1.data - jacobian_a_.data;
      changeJacRefPoint(jacobian_b_, v2, J_v2);
      J_v2.data=J_v2.data - jacobian_b_.data;

      changeJacDotRefPoint(jacobian_a_,jacobian_dot_a_,robot_state->kdl_jnt_array_vel_, v1, J_v1_dot);
      J_v1_dot.data = J_v1_dot.data - jacobian_dot_a_.data;
      changeJacDotRefPoint(jacobian_b_,jacobian_dot_b_,robot_state->kdl_jnt_array_vel_, v2, J_v2_dot);
      J_v2_dot.data = J_v2_dot.data - jacobian_dot_b_.data;

      Eigen::VectorXd qdot=robot_state->kdl_jnt_array_vel_.qdot.data;
      Eigen::Vector3d v1_dot = J_v1.data.topRows<3>()*qdot;
      Eigen::Vector3d v2_dot = J_v2.data.topRows<3>()*qdot;

      // VARIANT 1: e=acos(v1^T * v2) ==============================================
      /* Eigen::MatrixXd J__= Eigen::Map<const Eigen::Matrix<double,1,3> >(v2.data)*J_v1.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v1.data)*J_v2.data.topRows<3>(); */

      /* e_(0) = acos(dot(v1,v2)); //-delta */
      /* //regularize to avoid division-by-zero problems   */
      /* double eps=1e-5; */
      /* J_= -1/sqrt(1+eps-pow(dot(v1,v2),2))*J__; */
      /* e_dot_= J_*qdot; */
      /* J_dot_= -1/sqrt(1+eps-pow(dot(v1,v2),2))*(v2_dot.transpose()*J_v1.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v2.data)*J_v1_dot.data.topRows<3>()+v1_dot.transpose()*J_v2.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v1.data)*J_v2_dot.data.topRows<3>())-J__*dot(v1,v2)/pow(1+eps-pow(dot(v1,v2),2),1.5)*(dot(v1,KDL::Vector(v2_dot(0), v2_dot(1), v2_dot(2)))+dot(KDL::Vector(v1_dot(0),v1_dot(1),v1_dot(2)),v2)); */
      // END VARIANT 1 ==============================================================

      // VARIANT 2: e=v1^T * v2 -1 ==============================================
      e.resize(1);
      e(0) = dot(v1,v2)-1.0;
      J = Eigen::Map<const Eigen::Matrix<double,1,3> >(v2.data)*J_v1.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v1.data)*J_v2.data.topRows<3>();
      e_dot = J_*qdot;
      J_dot = v2_dot.transpose()*J_v1.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v2.data)*J_v1_dot.data.topRows<3>()+Eigen::Map<const Eigen::Matrix<double,1,3> >(v1.data)*J_v2_dot.data.topRows<3>()+v1_dot.transpose()*J_v2.data.topRows<3>();
      // END VARIANT 2 ==============================================================
      
      return 0;
    }

    
    template <typename PrimitiveA, typename PrimitiveB>
      int TDefTracking<PrimitiveA, PrimitiveB>::monitor() {
      return 0;
    }
 
 
    template <typename PrimitiveA, typename PrimitiveB>
      void TDefTracking<PrimitiveA, PrimitiveB>::maskJacobian(
							      RobotStatePtr robot_state) {
      for (unsigned int c = 0; c < robot_state->getNumJoints(); ++c) {
	if (!robot_state->isQNrWritable(c)){
	  J_.col(c).setZero();
	}
      }
    }
  
    template <typename PrimitiveA, typename PrimitiveB>
      void TDefTracking<PrimitiveA, PrimitiveB>::maskJacobianDerivative(
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
