#include <task_pop.h>
#include <hiqp_utils.h>

//#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>







namespace hiqp
{





TaskPoP::TaskPoP
(
	double n[3],
	double d
)
: d_(d)
{
	n_(0) = n[0];
	n_(1) = n[1];
	n_(2) = n[2];
}






bool TaskPoP::apply // this is for yumi
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel,
	double& task_fun_val,
	Eigen::MatrixXd& task_jac_val
)
{
	// Compute the pose of the end-effector
	KDL::TreeFkSolverPos_recursive fk_solver_pos(kdl_tree);
	KDL::Frame pose;
	int retval = fk_solver_pos.JntToCart(kdl_joint_pos_vel.q, 
		                                 pose,
		                                 "gripper_r_base");
	if (retval != 0)
	{
		std::cerr << "In TaskPoP::apply : Can't solve position of link "
			<< "'" << "gripper_r_base" << "'" << " in the KDL tree! "
			<< "KDL::TreeFkSolverPos_recursive::JntToCart return error code "
			<< "'" << retval << "'\n";
		return false;
	}
	Eigen::Vector3d p( pose.p.x(), pose.p.y(), pose.p.z() );

	// Compute the task jacobian dp/dq
	KDL::Jacobian jac;
	jac.resize(kdl_joint_pos_vel.q.rows());
	retval = kdl_JntToJac(kdl_tree, kdl_joint_pos_vel, jac, "gripper_r_base");
	if (retval != 0)
	{
		std::cerr << "In TaskPoP::apply : Can't solve jacobian of link "
			<< "'" << "gripper_r_base" << "'" << " in the KDL tree! "
			<< "hiqp::JntToJac return error code "
			<< "'" << retval << "'\n";
		return false;
	}

	// Set the task function and jacobian values
	task_fun_val = n_.dot(p);
	task_jac_val.resize(1, kdl_joint_pos_vel.q.rows());
	for (int i=0; i<kdl_joint_pos_vel.q.rows(); ++i)
		task_jac_val(0, i) = jac.getColumn(i).vel.z();

	return true;
}






bool apply_pendulum // this is for the pendulum robot
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel,
	double& task_fun_val,
	double& task_jac_val
)
{
/*
	KDL::Chain chain;
	kdl_tree.getChain("world", " link3", chain);

	KDL::ChainFkSolverVel_recursive fk_solver_vel(chain);
	KDL::FrameVel ee_pos_vel;
	fk_solver_vel.JntToCart(kdl_joint_pos_vel, ee_pos_vel);

	double p[3] = {
		ee_pos_vel.value()(0,3), 
		ee_pos_vel.value()(1,3), 
		ee_pos_vel.value()(2,3)
	};
	double dpdt[3] = {
		ee_pos_vel.deriv().vel(0), 
		ee_pos_vel.deriv().vel(1), 
		ee_pos_vel.deriv().vel(2)
	};
	double qdot = static_cast<double>( kdl_joint_pos_vel.deriv()(0) );

	double qdot_inv = (qdot>-0.001 && qdot<0.001 ? 0 : 1/qdot);

	double dpdq[3] = {
		dpdt[0]*qdot_inv,
		dpdt[1]*qdot_inv,
		dpdt[2]*qdot_inv
	};

	task_fun_val = n_[0]*p[0] + n_[1]*p[1] + n_[2]*p[2] - d_;
	task_jac_val = n_[0]*dpdq[0] + n_[1]*dpdq[1] + n_[2]*dpdq[2];


	task_fun_val = 0;
	task_jac_val = 0;
*/
	return true;

}








} // namespace hiqp