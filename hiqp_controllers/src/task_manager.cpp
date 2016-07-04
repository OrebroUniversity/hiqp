/*!
 * \file   task_manager.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/task_manager.h>
#include <hiqp/task_pop.h>
#include <hiqp/task_behaviour.h>
#include <hiqp/task_beh_fo.h>

// STL Includes
//#include <iostream>


#include <Eigen/Dense>









namespace hiqp {












TaskManager::TaskManager()
{
    TaskBehaviour* fobeh = new TaskBehFO(1.0);
    Task* poptask = new TaskPoP(fobeh, "gripper_r_base", 0, 0, 1, 0.1);
    tasks_.push_back(poptask);
}


TaskManager::~TaskManager() noexcept
{
    // We have memory leaks!!
}


bool TaskManager::getKinematicControls
(
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel,
	std::vector<double> &controls
)
{
    tasks_.at(0)->getControls(kdl_tree, 
                              kdl_joint_pos_vel, 
                              controls);

     
    //std::cout << tasks_.at(0)->getJ() << std::endl;

    /*
    std::cout << "controls = ";
    for (double c : controls)
    {
         std::cout << c << ", ";
    }
    std::cout << "\n";
    */

    return true;
}











} // namespace hiqp


