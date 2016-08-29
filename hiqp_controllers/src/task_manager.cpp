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

    task_factory_ = new TaskFactory();
}


/*!
 * \todo cleanup of tasks and task dynamics is missing!
 */
TaskManager::~TaskManager() noexcept
{
    delete solver_;
    delete geometric_primitive_map_;
    delete task_factory_;
}




void TaskManager::init
(
    unsigned int num_controls
)
{ 
    num_controls_ = num_controls; 
    task_factory_->init(geometric_primitive_map_, visualizer_, num_controls_);
}







bool TaskManager::getKinematicControls
(
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel,
    std::vector<double> &controls
)
{
    if (tasks_.size() < 1) 
    {
        for (int i=0; i<controls.size(); ++i)
            controls.at(i) = 0;
        
        return false;
    }

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

    geometric_primitive_map_->redrawAllPrimitives();

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







int TaskManager::addTask
(
    const std::string& name,
    const std::string& type,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters
)
{

    TaskDynamics* dynamics = nullptr;
    TaskFunction* function = nullptr;

    dynamics = task_factory_->buildTaskDynamics(behaviour_parameters);
    if (dynamics == nullptr)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', could not parse the dynamics parameters! No task was added!");
        return -1;
    }

    task_dynamics_[next_task_dynamics_id_] = dynamics;
    next_task_dynamics_id_++;

    function = task_factory_->buildTaskFunction(
        name, next_task_id_, type, priority, visibility, parameters, dynamics);

    if (function == nullptr)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', could not parse the task parameters! No task was added!");
        delete dynamics;
        return -3;
    }

    bool size_test1 = (function->e_.rows() != function->J_.rows());
    bool size_test2 = (function->e_.rows() != function->e_dot_star_.rows());
    bool size_test3 = (function->e_.rows() != function->task_types_.size());

    if (size_test1 || size_test2 || size_test3)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', the task dimensions was not properly setup! No task was added!");
    }

    tasks_.insert( TaskMapElement(next_task_id_, function) );
    next_task_id_++;

    return next_task_id_-1;
}






/*! \todo removal of tasks must also delete task dynamics or else there's a memory leak!
 */
int TaskManager::removeTask
(
    std::size_t task_id
)
{
    if (tasks_.erase(task_id) == 1) 
    {
        geometric_primitive_map_->removeDependency(task_id);
        return 0;
    }

    return -1;
}





int TaskManager::removeAllTasks
()
{
    TaskMapIterator it = tasks_.begin();
    while (it != tasks_.end())
    {
        geometric_primitive_map_->removeDependency(it->first);
        ++it;
    }

    tasks_.clear();
    task_dynamics_.clear();
    return 0;
}





int TaskManager::addGeometricPrimitive
(
    const std::string& name,
    const std::string& type,
    const std::string& frame_id,
    bool visible,
    const std::vector<double>& color,
    const std::vector<double>& parameters
)
{

    geometric_primitive_map_->addGeometricPrimitive(
        name, type, frame_id, visible, color, parameters);

    return 0;
}





int TaskManager::removeGeometricPrimitive
(
    std::string name
)
{
    return geometric_primitive_map_->removeGeometricPrimitive(name);
}




int TaskManager::removeAllGeometricPrimitives
()
{
    geometric_primitive_map_->clear();
}








/*
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
*/






} // namespace hiqp


