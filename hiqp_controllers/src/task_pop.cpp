#include <task_pop.h>

#include <kdl/chainfksolvervel_recursive.hpp>






namespace hiqp
{





TaskPoP::TaskPoP
(
	double n[3],
	double d
)
: d_(d)
{
	n_[0] = n[0];
	n_[1] = n[1];
	n_[2] = n[2];
}






bool TaskPoP::apply
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel,
	double& task_fun_val,
	double& task_jac_val
)
{

	KDL::Chain chain;
	kdl_tree.getChain("world", "link3", chain);

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

	return true;
}






} // namespace hiqp