#include <task_pop.h>
#include <hiqp_utils.h>

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






bool TaskPoP::apply // this is for yumi
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel,
	double& task_fun_val,
	double& task_jac_val
)
{

/*
	std::cout << "kdl_tree = (" << kdl_tree << ")\n";
	//outputs:
	kdl_tree = (gripper_l_base, 
		        gripper_l_finger_l, 
		        gripper_l_finger_r, 
		        gripper_r_base, 
		        gripper_r_finger_l, 
		        gripper_r_finger_r, 
		        world, 
		        yumi_body, 
		        yumi_link_1_l, 
		        yumi_link_1_r, 
		        yumi_link_2_l, 
		        yumi_link_2_r, 
		        yumi_link_3_l, 
		        yumi_link_3_r, 
		        yumi_link_4_l, 
		        yumi_link_4_r, 
		        yumi_link_5_l, 
		        yumi_link_5_r, 
		        yumi_link_6_l, 
		        yumi_link_6_r, 
		        yumi_link_7_l, 
		        yumi_link_7_r)
*/


	KDL::Chain chain;
	kdl_tree.getChain("world", " gripper_r_base", chain);

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
	//std::cout << "N_joints: " << kdl_joint_pos_vel.value().columns() << "\n";
	/*
	double qdot = static_cast<double>( kdl_joint_pos_vel.deriv()(0) );

	double qdot_inv = (qdot>-0.001 && qdot<0.001 ? 0 : 1/qdot);

	double dpdq[3] = {
		dpdt[0]*qdot_inv,
		dpdt[1]*qdot_inv,
		dpdt[2]*qdot_inv
	};

	task_fun_val = n_[0]*p[0] + n_[1]*p[1] + n_[2]*p[2] - d_;
	task_jac_val = n_[0]*dpdq[0] + n_[1]*dpdq[1] + n_[2]*dpdq[2];
*/

	task_fun_val = 0;
	task_jac_val = 0;

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