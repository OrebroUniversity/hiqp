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
 
#include <hiqp/hiqp_utils.h>

#include <hiqp/tasks/task_geometric_projection.h>
#include <hiqp/tasks/dynamics_first_order.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_plane.h>

// STL Includes
//#include <iostream>
#include <cassert>


#include <Eigen/Dense>









namespace hiqp {












TaskManager::TaskManager
(
    Visualizer* visualizer
)
: next_task_id_(0), 
  next_task_dynamics_id_(0),
  visualizer_(visualizer)
{
    solver_ = new CasADiSolver();
    geometric_primitive_map_ = new GeometricPrimitiveMap(visualizer_);
}


TaskManager::~TaskManager() noexcept
{
    // We have memory leaks in tasks_ and task_behaviours_ !!!
    delete solver_;
    delete geometric_primitive_map_;
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

    //task_visualizer_->redraw();

    return true;
}



void TaskManager::getTaskMonitoringData
(
    std::vector<TaskMonitoringData>& data
)
{
    for (TaskMapElement&& element : tasks_)
    {
        element.second->monitor(); // computes the performance measures
        data.push_back( 
            TaskMonitoringData
            (
                element.second->getId(),
                element.second->getTaskName(),
                element.second->performance_measures_
            )
        );
    }
}







std::size_t TaskManager::addTask
(
    const std::string& name,
    const std::string& type,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters
)
{


    // Create the task behaviour
    TaskDynamics* dynamics = nullptr;
    if (behaviour_parameters.size() == 1 && behaviour_parameters.at(0).compare("NA") == 0)
    {
        dynamics = buildTaskDynamics("DynamicsFirstOrder");
        dynamics->init( {"DynamicsFirstOrder", "1.0"} );
    }
    else if (behaviour_parameters.size() >= 2)
    {
        dynamics = buildTaskDynamics(behaviour_parameters.at(0));
        if (dynamics == nullptr)
            return -2;
        dynamics->init(behaviour_parameters);
    }
    else
    {
        return -1;
    }

    // Add the task dynamics to the behaviours map
    task_dynamics_[next_task_dynamics_id_] = dynamics;
    next_task_dynamics_id_++;

    // Create and initialize the task
    TaskFunction* task = buildTaskFunction(type, parameters);
    if (task == nullptr)
    {
        delete dynamics;
        return -3;
    }

    // Initialize the task
    task->setTaskDynamics(dynamics);
    task->setVisualizer(visualizer_);
    task->setId(next_task_id_);
    task->setTaskName(name);
    task->setPriority(priority);
    task->setVisibility(visibility);
    task->setGeometricPrimitiveMap(geometric_primitive_map_);

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




int TaskManager::addGeometricPrimitive
(
    const std::string& name,
    const std::string& type,
    const std::string& frame_id,
    bool visible,
    const std::vector<double>& color,
    const std::vector<std::string>& parameters
)
{

    geometric_primitive_map_->addGeometricPrimitive(
        name, type, frame_id, visible, color, parameters);

    return 0;
}









TaskFunction* TaskManager::buildTaskFunction
(
    const std::string& type,
    const std::vector<std::string>& parameters
)
{
    if (type.compare("TaskGeometricProjection") == 0)
    {
        if (parameters.at(0).compare("point") == 0 && 
            parameters.at(1).compare("plane") == 0)
        {
            return new TaskGeometricProjection<GeometricPoint, GeometricPlane>();
        }

    }

    return nullptr;
}





TaskDynamics* TaskManager::buildTaskDynamics
(
    const std::string& behaviour_name
)
{
    if (behaviour_name.compare("DynamicsFirstOrder") != 0)
        return nullptr;

    return new DynamicsFirstOrder();
}







} // namespace hiqp


