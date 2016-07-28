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
 * \file   task_pop.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/impl/task_pop.h>
#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

#include <iostream>






namespace hiqp
{





TaskPoP::TaskPoP() {}






int TaskPoP::init
(
    const std::vector<std::string>& parameters,
    unsigned int numControls
)
{
	if (parameters.size() != 9)
		return -1;

	point_frame_id_ = parameters.at(0);
	p_(0) = std::stod( parameters.at(1) );
	p_(1) = std::stod( parameters.at(2) );
	p_(2) = std::stod( parameters.at(3) );

	plane_frame_id_ = parameters.at(4);
	n_(0) = std::stod( parameters.at(5) );
	n_(1) = std::stod( parameters.at(6) );
	n_(2) = std::stod( parameters.at(7) );
	d_ = std::stod( parameters.at(8) );

	if (getVisibility())
	{
		getTaskVisualizer()->createSphere(point_frame_id_, 
			p_(0), p_(1), p_(2), 0.005, 1.0, 0.0, 0.0, 0.9);

		getTaskVisualizer()->createPlane(plane_frame_id_, 
			n_(0), n_(1), n_(2), d_, 1.0, 0.0, 0.0, 0.9);
	}

	e_.resize(1);
	J_.resize(1, numControls);
	e_dot_star_.resize(1);
	task_types_.insert(task_types_.begin(), 1, 0); // a 1-D eq task

	performance_measures_.resize(1);

	std::cout << "TaskPoP::init finished successfully\n";
}







int TaskPoP::apply
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel
)
{
	
	// Compute the pose of the end-effector
	KDL::TreeFkSolverPos_recursive fk_solver_pos(kdl_tree);

	KDL::Frame pose;

	int retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
		                                 pose,
		                                 point_frame_id_);

	if (retval != 0)
	{
		std::cerr << "In TaskPoP::apply : Can't solve position of link "
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
		std::cerr << "In TaskPoP::apply : Can't solve position of link "
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
		std::cerr << "In TaskPoP::apply : Can't solve jacobian of link "
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
}




int TaskPoP::monitor()
{
	performance_measures_.at(0) = e_(0);
	
	return 0;
}





} // namespace hiqp