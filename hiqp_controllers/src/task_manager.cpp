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

#include <hiqp/hiqp_utils.h>

// STL Includes
//#include <iostream>


#include <Eigen/Dense>









namespace hiqp {












TaskManager::TaskManager(TaskVisualizer* task_visualizer)
: next_task_id_(0), 
  next_task_behaviour_id_(0),
  task_visualizer_(task_visualizer)
{
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
    if (tasks_.size() < 1) return true;

    tasks_.at(0)->computeTaskMetrics(kdl_tree, 
                                     kdl_joint_pos_vel);

    double e_dot_star = tasks_.at(0)->e_dot_star_;
    Eigen::MatrixXd J = tasks_.at(0)->J_;

    Eigen::MatrixXd u = e_dot_star * dls(J, 0.01);
    for (int i= 0; i<controls.size(); ++i)
    {
        controls[i] = u(i);
    }

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








std::size_t TaskManager::addTask
(
    const std::string& task_name,
    const std::string& behaviour_name,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters
)
{
    // Create the task behaviour
    TaskBehaviour* behaviour = buildTaskBehaviour(behaviour_name);
    if (behaviour == NULL)
        return -1;

    // Initialize the task behaviour
    behaviour->init(behaviour_parameters);

    // Add the task behaviour to the behaviours map
    task_behaviours_[next_task_behaviour_id_] = behaviour;
    next_task_behaviour_id_++;

    // Create and initialize the task
    Task* task = buildTask(task_name);
    if (task == NULL)
    {
        delete behaviour;
        return -1;
    }

    // Initialize the task
    task->setTaskBehaviour(behaviour);
    task->setTaskVisualizer(task_visualizer_);
    task->setPriority(priority);
    task->setVisibility(visibility);
    task->init(parameters);

    // Add the task to the tasks map
    tasks_[next_task_id_] = task;
    next_task_id_++;

    return next_task_id_-1;
}







int TaskManager::removeTask
(
    std::size_t task_id
)
{

    if (tasks_.find(task_id) == tasks_.end())
        return -1;

    tasks_.erase(task_id);
    return 0;
}









Task* TaskManager::buildTask
(
    const std::string& task_name
)
{
    if (task_name.compare("TaskPoP") != 0)
        return NULL;

    return new TaskPoP();
}





TaskBehaviour* TaskManager::buildTaskBehaviour
(
    const std::string& behaviour_name
)
{
    if (behaviour_name.compare("TaskBehFO") != 0)
        return NULL;

    return new TaskBehFO();
}







} // namespace hiqp


