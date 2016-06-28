#include <task_manager.h>

#include <iostream>
/*
#include <string>
#include <map>
#include <vector>
*/
//#include <casadi/casadi.hpp>

#include <kdl/tree.hpp>











namespace hiqp {












TaskManager::TaskManager()
{}


TaskManager::~TaskManager() noexcept
{}


bool TaskManager::getKinematicControls
(
     const KDL::Tree& kdl_tree,
     const KDL::JntArray& kdl_joint_pos,
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

     //std::vector<casadi::SX> n = {0, 0, 1};



     //std::map<std::string, KDL::TreeElement>::const_iterator link2;
     //link2 = kdl_tree.getSegment("link2");

     //KDL::Chain chain;
     //kdl_tree.getChain("world", "link2", chain);
     //KDL::ChainFkSolverPos_recursive fk_solver(chain);


     for (auto&& control : controls)
          control = 1;












     return true;
}











} // namespace hiqp


