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
//#include <kdl/chainfksolvervel_recursive.hpp>











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
/*
     KDL::Chain chain;
     kdl_tree.getChain("world", "link3", chain);

     KDL::ChainFkSolverPos_recursive fk_solver_pos(chain);
     KDL::Frame ee_pos;
     fk_solver_pos.JntToCart(kdl_joint_pos, ee_pos);

     KDL::ChainFkSolverVel_recursive fk_solver_vel(chain);
     KDL::FrameVel ee_vel_tmp;
     //KDL::JntArray kdl_joint_vel = kdl_joint_states.qdot;
     fk_solver_vel.JntToCart(kdl_joint_states, ee_vel_tmp);
     KDL::Frame ee_vel = ee_vel_tmp.value();


     std::vector<casadi::SX> n = {0, 0, 1};
     std::vector<casadi::SX> p = {ee_pos(0,3), ee_pos(1,3), ee_pos(2,3)};
     double lambda = 1;
     double d = 1.2;
     std::vector<casadi::SX> dpdq = {ee_vel(0,3), ee_vel(1,3), ee_vel(2,3)};

     std::cout << "Vel = (" 
     << ee_vel(0,3) << ", " 
     << ee_vel(1,3) << ", " 
     << ee_vel(2,3) << ")\n";
*/
     for (auto&& control : controls)
          control = 1;












     return true;
}











} // namespace hiqp


