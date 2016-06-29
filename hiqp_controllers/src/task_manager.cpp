#include <task_manager.h>
#include <task_pop.h>

// STL Includes
#include <iostream>

// CasADi includes
#include <casadi/casadi.hpp>











namespace hiqp {












TaskManager::TaskManager()
{
     double n[3] = {0,0,1};
     boost::shared_ptr<Task> poptask( new TaskPoP(n, 1.2) );
     tasks_.push_back(poptask);
}


TaskManager::~TaskManager() noexcept
{
     // We have a memory leak!!
}


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

     double task_fun_val;
     double task_jac_val;
     tasks_.at(0)->apply(kdl_tree, 
                         kdl_joint_pos_vel, 
                         task_fun_val, 
                         task_jac_val);



     double lambda = 1;
     double J_inv = (task_jac_val==0 ? 1 : 1/task_jac_val);

     double u = -lambda * J_inv * task_fun_val;

     //std::cout << " u = " << u << "\n";
     
     controls.at(0) = u;







     return true;
}











} // namespace hiqp


