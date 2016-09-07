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
: //next_task_id_(0), 
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
    const std::chrono::steady_clock::time_point& sampling_time,
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
        element.second->computeTaskMetrics(sampling_time,
                                           kdl_tree,
                                           kdl_joint_pos_vel);

        solver_->appendStage(element.second->priority_, 
                             element.second->e_dot_star_, 
                             element.second->J_,
                             element.second->task_types_);
    }

    solver_->solve(controls);
    
    return true;
}



void TaskManager::getTaskMonitoringData
(
    std::vector<TaskMonitoringData>& data
)
{
    for (TaskMapElement&& element : tasks_)
    {
        // computes the custom performance measures
        element.second->monitor();

        // copies the e and e_dot_star_ values onto double-vectors
        element.second->monitorFunctionAndDynamics();

        data.push_back( 
            TaskMonitoringData
            (
                element.second->getTaskName(),
                "e",
                element.second->measures_e_
            )
        );

        data.push_back( 
            TaskMonitoringData
            (
                element.second->getTaskName(),
                "de*",
                element.second->measures_e_dot_star_
            )
        );

        data.push_back( 
            TaskMonitoringData
            (
                element.second->getTaskName(),
                "PM",
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
    const std::vector<std::string>& parameters,
    const std::chrono::steady_clock::time_point& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel
)
{

    TaskDynamics* dynamics = nullptr;
    TaskFunction* function = nullptr;

    int result = task_factory_->buildTask(
        name, 
        //next_task_id_, 
        type, 
        priority, 
        visibility, 
        parameters, 
        behaviour_parameters,
        sampling_time,
        kdl_tree,
        kdl_joint_pos_vel,
        next_task_dynamics_id_,
        dynamics,
        function
    );

    if (result == 0)
    {
        task_dynamics_[next_task_dynamics_id_] = dynamics;
        next_task_dynamics_id_++;
        tasks_.insert( TaskMapElement(name, function) );

        printHiqpInfo("Added task '" + name + "'.");

        return 0;
    }

    return -1;
}





int TaskManager::updateTask
(
    const std::string& name,
    const std::string& type,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters,
    const std::chrono::steady_clock::time_point& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel
)
{

    TaskDynamics* dynamics = nullptr;
    TaskFunction* function = nullptr;

    int result = 0;//task_factory_->updateTask(
    //     name, 
    //     next_task_id_, 
    //     type, 
    //     priority, 
    //     visibility, 
    //     parameters, 
    //     behaviour_parameters,
    //     sampling_time,
    //     kdl_tree,
    //     kdl_joint_pos_vel,
    //     dynamics,
    //     function
    // );

    if (result == 0)
    {
        printHiqpInfo("Updated task '" + name + "'.");
        return 0;
    }
    else
    {
        printHiqpWarning("Couldn't update task '" + name + "'.");
    }

    return -1;
}





/*! \todo removal of tasks must also delete task dynamics or else there's a memory leak!
 */
int TaskManager::removeTask
(
    std::string task_name
)
{
    if (tasks_.erase(task_name) == 1) 
    {
        geometric_primitive_map_->removeDependency(task_name);
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












} // namespace hiqp


