/*!
 * \file   task_pop.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/task_pop.h>
#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>

#include <iostream>






namespace hiqp
{





TaskPoP::TaskPoP
(
	TaskBehaviour* behaviour,
	std::string link_name,
	double nx, 
	double ny, 
	double nz,
	double d
)
: Task(behaviour), link_name_(link_name), d_(d)
{
	n_(0) = nx;
	n_(1) = ny;
	n_(2) = nz;
}






int TaskPoP::apply // this is for yumi
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
		                                 link_name_);
	if (retval != 0)
	{
		std::cerr << "In TaskPoP::apply : Can't solve position of link "
			<< "'" << link_name_ << "'" << " in the KDL tree! "
			<< "KDL::TreeFkSolverPos_recursive::JntToCart return error code "
			<< "'" << retval << "'\n";
		return -1;
	}
	Eigen::Vector3d p( pose.p.x(), pose.p.y(), pose.p.z() );

	// Compute the task jacobian dp/dq
	KDL::Jacobian jac;
	jac.resize(kdl_joint_pos_vel.q.rows());
	retval = kdl_JntToJac(kdl_tree, kdl_joint_pos_vel, jac, link_name_);
	if (retval != 0)
	{
		std::cerr << "In TaskPoP::apply : Can't solve jacobian of link "
			<< "'" << link_name_ << "'" << " in the KDL tree! "
			<< "hiqp::JntToJac return error code "
			<< "'" << retval << "'\n";
		return -2;
	}

	// Set the task function and jacobian values
	e = n_.dot(p);
	J.resize(1, kdl_joint_pos_vel.q.rows());
	for (int i=0; i<kdl_joint_pos_vel.q.rows(); ++i)
		J(0, i) = jac.getColumn(i).vel.z();


    //std::cout << "J = " << J << "\n";

	return 0;
}









} // namespace hiqp