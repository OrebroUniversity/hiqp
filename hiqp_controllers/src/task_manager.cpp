#include <task_manager.h>

#include <iostream>
/*
#include <string>
#include <map>
#include <vector>
*/
#include <casadi/casadi.hpp>

// Orocos KDL
//#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>











namespace hiqp {












TaskManager::TaskManager()
{}


TaskManager::~TaskManager() noexcept
{}


bool TaskManager::getKinematicControls
(
     const KDL::Tree& kdl_tree,
     const KDL::JntArrayVel& kdl_joint_pos_vel,
	unsigned int n_controls,
	std::vector<double> &controls
)
{
     
     if (n_controls != controls.size())
     {
          std::cerr << "In TaskManager::getKinematicControls, size of"
               << " controls do not match n_controls. Aborting!\n";
          return false;
     }





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

     double n[3] = {0, 0, 1};
     double lambda = 1;
     double d = 1.2;
     
     double e = n[0]*p[0] + n[1]*p[1] + n[2]*p[2] - d;
     double J = n[0]*dpdq[0] + n[1]*dpdq[1] + n[2]*dpdq[2];
     double J_inv = (J==0 ? 1 : 1/J);

     double u = -lambda * J_inv * e;

     std::cout << " u = " << u << "\n";
     
     controls.at(0) = u;







     return true;
}











} // namespace hiqp


