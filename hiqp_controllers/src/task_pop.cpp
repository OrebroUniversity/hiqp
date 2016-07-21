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





TaskPoP::TaskPoP() {}






int TaskPoP::init(const std::vector<std::string>& parameters)
{
	if (parameters.size() != 6)
		return -1;

	link_name_ = parameters.at(0);
	base_link_name_ = parameters.at(1);
	n_(0) = std::stod( parameters.at(2) );
	n_(1) = std::stod( parameters.at(3) );
	n_(2) = std::stod( parameters.at(4) );
	d_ = std::stod( parameters.at(5) );

	std::cout << "TaskPoP::init\n";

	getTaskVisualizer()->createPlane(base_link_name_, n_(0), n_(1), n_(2), d_, 0.85, 0.0, 0.0, 0.5);
	getTaskVisualizer()->createSphere(link_name_, 0.0, 0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.85);
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

	//std::cout << "here\n";

	// Set the task function and jacobian values
	e_.resize(1, 1);
	e_(0, 0) = n_.dot(p);
	J_.resize(1, kdl_joint_pos_vel.q.rows());
	for (int i=0; i<kdl_joint_pos_vel.q.rows(); ++i)
		J_(0, i) = jac.getColumn(i).vel.z();

    //std::cout << "J = " << J << "\n";

	return 0;
}




int TaskPoP::draw()
{
	// Update all visual primitives used by this task
}





} // namespace hiqp