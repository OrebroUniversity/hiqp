// The HiQP Control Framework, an optimal control framework targeted at robotics
// Copyright (C) 2016 Marcus A Johansson
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.




/*!
 * \file   task_manager.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/task_manager.h>
#include <hiqp/impl/task_pop.h>
#include <hiqp/task_behaviour.h>
#include <hiqp/impl/task_beh_fo.h>

#include <hiqp/hiqp_utils.h>

// STL Includes
//#include <iostream>
#include <cassert>


#include <Eigen/Dense>









namespace hiqp {












TaskManager::TaskManager
(
    TaskVisualizer* task_visualizer
)
: next_task_id_(0), 
  next_task_behaviour_id_(0),
  task_visualizer_(task_visualizer)
{
    solver_ = new CasADiSolver();
}


TaskManager::~TaskManager() noexcept
{
    // We have memory leaks in tasks_ and task_behaviours_ !!!
    delete solver_;
}







bool TaskManager::getKinematicControls
(
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel,
    std::vector<double> &controls
)
{
    if (tasks_.size() < 1) return false;

    solver_->clearStages();

    for (TaskMapElement&& element : tasks_)
    {
        element.second->computeTaskMetrics(kdl_tree,
                                           kdl_joint_pos_vel);

        solver_->appendStage(element.second->priority_, 
                             element.second->e_dot_star_, 
                             element.second->J_,
                             element.second->task_types_);
    }

    solver_->solve(controls);

    task_visualizer_->redraw();

    return true;
}







std::size_t TaskManager::addTask
(
    const std::string& task_type,
    const std::string& behaviour_type,
    const std::vector<std::string>& behaviour_parameters,
    const std::string& task_name,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters
)
{
    // Create the task behaviour
    TaskBehaviour* behaviour = buildTaskBehaviour(behaviour_type);
    if (behaviour == NULL)
        return -1;

    // Initialize the task behaviour
    behaviour->init(behaviour_parameters);

    // Add the task behaviour to the behaviours map
    task_behaviours_[next_task_behaviour_id_] = behaviour;
    next_task_behaviour_id_++;

    // Create and initialize the task
    Task* task = buildTask(task_type);
    if (task == NULL)
    {
        delete behaviour;
        return -1;
    }

    // Initialize the task
    task->setTaskBehaviour(behaviour);
    task->setTaskVisualizer(task_visualizer_);
    task->setId(next_task_id_);
    task->setTaskName(task_name);
    task->setPriority(priority);
    task->setVisibility(visibility);
    task->init(parameters, numControls_);

    assert(task->e_.rows() == task->J_.rows());
    assert(task->e_.rows() == task->e_dot_star_.rows());
    assert(task->e_.rows() == task->task_types_.size());

    // Add the task to the tasks map
    tasks_.insert( TaskMapElement(next_task_id_, task) );
    next_task_id_++;

    return next_task_id_-1;
}







int TaskManager::removeTask
(
    std::size_t task_id
)
{
    if (tasks_.erase(task_id) == 1) 
        return 0;

    return -1;
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


