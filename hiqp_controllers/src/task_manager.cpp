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

     casadi::SX s;

     //std::vector<casadi::SX> n = {0, 0, 1};
 /*    std::vector<casadi::SX> p = {
          ee_pos_vel.value()(0,3), 
          ee_pos_vel.value()(1,3), 
          ee_pos_vel.value()(2,3)};
     double lambda = 1;
     double d = 1.2;
     //std::vector<casadi::SX> dpdq = {ee_vel(0,3), ee_vel(1,3), ee_vel(2,3)};

     std::cout << "pos = (" 
     << p[0] << ", " 
     << p[1] << ", " 
     << p[2] << ")\n";
*/
     for (auto&& control : controls)
          control = 1;












     return true;
}











} // namespace hiqp


