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




/*!
 * \file   task_geometric_projection__impl.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_GEOMETRIC_PROJECTION__IMPL_H
#define HIQP_TASK_GEOMETRIC_PROJECTION__IMPL_H


// Orocos KDL Includes
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

#include <sstream>
#include <iterator>





namespace hiqp
{








template<typename PrimitiveA, typename PrimitiveB>
TaskGeometricProjection<PrimitiveA, PrimitiveB>::TaskGeometricProjection()
{}




template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricProjection<PrimitiveA, PrimitiveB>::init
(
	const std::chrono::steady_clock::time_point& sampling_time,
    const std::vector<std::string>& parameters,
    const KDL::Tree& kdl_tree, 
    unsigned int num_controls
)
{

	if (parameters.size() != 3)
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
	if (primitive_a_ == nullptr)
	{
		printHiqpWarning("In TaskGeometricProjection::init(), couldn't find primitive with name '"
			+ args.at(0) + "'. Unable to create task!");
		return -3;
	}

	primitive_b_ = geometric_primitive_map_->getGeometricPrimitive<PrimitiveB>(args.at(2));
	if (primitive_b_ == nullptr)
	{
		printHiqpWarning("In TaskGeometricProjection::init(), couldn't find primitive with name '"
			+ args.at(2) + "'. Unable to create task!");
		return -3;
	}

	geometric_primitive_map_->addDependencyToPrimitive(args.at(0), this->getId());
	geometric_primitive_map_->addDependencyToPrimitive(args.at(2), this->getId());

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
		return -4;
	}

	task_types_.insert(task_types_.begin(), 1, sign);

	return 0;

/*
	point_frame_id_ = parameters.at(0);
	p_(0) = std::stod( parameters.at(1) );
	p_(1) = std::stod( parameters.at(2) );
	p_(2) = std::stod( parameters.at(3) );

	plane_frame_id_ = parameters.at(4);
	n_(0) = std::stod( parameters.at(5) );
	n_(1) = std::stod( parameters.at(6) );
	n_(2) = std::stod( parameters.at(7) );
	d_ = std::stod( parameters.at(8) );
*/

	//std::cout << "TaskGeometricProjection::init finished successfully\n";
}




template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricProjection<PrimitiveA, PrimitiveB>::monitor()
{
	performance_measures_.at(0) = e_(0);
	
	return 0;
}






template<typename PrimitiveA, typename PrimitiveB>
int TaskGeometricProjection<PrimitiveA, PrimitiveB>::apply
(
	const std::chrono::steady_clock::time_point& sampling_time,
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
		std::cerr << "In TaskGeometricProjection::apply : Can't solve position "
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
		std::cerr << "In TaskGeometricProjection::apply : Can't solve position "
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
		std::cerr << "In TaskGeometricProjection::apply : Can't solve jacobian "
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
		std::cerr << "In TaskGeometricProjection::apply : Can't solve jacobian "
			<< "of link '" << primitive_b_->getFrameId() << "'" << " in the "
			<< "KDL tree! KDL::TreeJntToJacSolver return error code "
			<< "'" << retval << "'\n";
		return -4;
	}



	return project(primitive_a_, primitive_b_);

/*









	// Compute the pose of the end-effector
	KDL::TreeFkSolverPos_recursive fk_solver_pos(kdl_tree);

	KDL::Frame pose;

	int retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
		                                 pose,
		                                 point_frame_id_);

	if (retval != 0)
	{
		std::cerr << "In TaskGeometricProjection::apply : Can't solve position of link "
			<< "'" << point_frame_id_ << "'" << " in the KDL tree! "
			<< "KDL::TreeFkSolverPos_recursive::JntToCart return error code "
			<< "'" << retval << "'\n";
		return -1;
	}

	KDL::Vector p__ = pose.M * p_;

	// this gives a vector to the end-effector in root coordinates
	KDL::Vector p( pose.p.x() + p__(0), 
		           pose.p.y() + p__(1), 
		           pose.p.z() + p__(2) );





	// Compute the pose of the plane
	retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
	                                 pose,
	                                 plane_frame_id_);

	if (retval != 0)
	{
		std::cerr << "In TaskGeometricProjection::apply : Can't solve position of link "
			<< "'" << plane_frame_id_ << "'" << " in the KDL tree! "
			<< "KDL::TreeFkSolverPos_recursive::JntToCart return error code "
			<< "'" << retval << "'\n";
		return -1;
	}

	// this gives the plane normal in root coordinates
	KDL::Vector n = pose.M * n_;

	// this gives the offset from the plane to the root origo
	double d = d_ + KDL::dot(n, pose.p);





	// Compute the task jacobian dp/dq in root coordinates
	KDL::Jacobian jac;

	jac.resize(kdl_joint_pos_vel.q.rows());

	KDL::TreeJntToJacSolver fk_solver_jac(kdl_tree);

	retval = fk_solver_jac.JntToJac(kdl_joint_pos_vel.q,
		                            jac,
		                            point_frame_id_);

	//retval = kdl_JntToJac(kdl_tree, kdl_joint_pos_vel, jac, point_frame_id_);

	if (retval != 0)
	{
		std::cerr << "In TaskGeometricProjection::apply : Can't solve jacobian of link "
			<< "'" << point_frame_id_ << "'" << " in the KDL tree! "
			<< "KDL::TreeJntToJacSolver return error code "
			<< "'" << retval << "'\n";
		return -2;
	}





	// Set the task function and jacobian values

	e_(0) = KDL::dot(n, p) - d;

	for (int q_nr = 0; q_nr < kdl_joint_pos_vel.q.rows(); ++q_nr)
	{
		J_(0, q_nr) = n(0) * jac.getColumn(q_nr).vel.x() + 
		              n(1) * jac.getColumn(q_nr).vel.y() + 
		              n(2) * jac.getColumn(q_nr).vel.z();      
	}





    //std::cout << "e = " << e_ << "\n\n";
    //std::cout << "J = " << J_ << "\n\n";

	return 0;

	*/
}






template<typename PrimitiveA, typename PrimitiveB>
KDL::Vector TaskGeometricProjection<PrimitiveA, PrimitiveB>
::getVelocityJacobianForTwoPoints
(
    const KDL::Vector& p1, 
    const KDL::Vector& p2,
    int q_nr
)
{
	KDL::Twist Ja = jacobian_a_.getColumn(q_nr);
	KDL::Twist Jb = jacobian_b_.getColumn(q_nr);
	KDL::Vector Jp1 = Ja.rot * p1;
	KDL::Vector Jp2 = Jb.rot * p2;

	return ( Jb.vel+Jp2 - (Ja.vel+Jp1) );
}











} // namespace hiqp

#endif // include guard