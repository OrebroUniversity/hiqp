#include <task_manager.h>
#include <task_pop.h>
#include <hiqp_utils.h>

// STL Includes
#include <iostream>

// CasADi includes
#include <casadi/casadi.hpp>











namespace hiqp {












TaskManager::TaskManager()
{
     double n[3] = {0,0,1};
     boost::shared_ptr<Task> poptask( new TaskPoP(n, 0.1) );
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
	std::vector<double> &controls
)
{

     double task_fun_val;
     Eigen::MatrixXd task_jac_val;
     
     tasks_.at(0)->apply(kdl_tree, 
                         kdl_joint_pos_vel, 
                         task_fun_val, 
                         task_jac_val);



     double lambda = 1;
     
     Eigen::MatrixXd Jinv;
     Jinv.resizeLike(task_jac_val);
     pseudoInverse<Eigen::MatrixXd>(task_jac_val, Jinv);


     Eigen::MatrixXd u;
     u.resizeLike(Jinv);
     u = -lambda * Jinv * task_fun_val;

     //std::cout << "u = " << u << "\n";

     for (int i=0; i<controls.size(); ++i)
          controls.at(i) = u(0,i);







     return true;
}











} // namespace hiqp


