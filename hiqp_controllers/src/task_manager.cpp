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

 #include <hiqp/geometric_primitives/geometric_point.h>
 #include <hiqp/geometric_primitives/geometric_plane.h>

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
    TaskBehaviour* behaviour = nullptr;
    if (behaviour_parameters.size() == 1 && behaviour_parameters.at(0).compare("NA") == 0)
    {
        behaviour = buildTaskBehaviour("TaskBehFO");
        behaviour->init( {"TaskBehFO", "1.0"} );
    }
    else if (behaviour_parameters.size() >= 2)
    {
        behaviour = buildTaskBehaviour(behaviour_parameters.at(0));
        if (behaviour == NULL)
            return -2;
        behaviour->init(behaviour_parameters);
    }
    else
    {
        return -1;
    }

    // Add the task behaviour to the behaviours map
    task_behaviours_[next_task_behaviour_id_] = behaviour;
    next_task_behaviour_id_++;

    // Create and initialize the task
    Task* task = buildTask(type);
    if (task == NULL)
    {
        delete behaviour;
        return -3;
    }

    // Initialize the task
    task->setTaskBehaviour(behaviour);
    task->setTaskVisualizer(task_visualizer_);
    task->setId(next_task_id_);
    task->setTaskName(name);
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


    /*
# Available types and parameter list syntaxes

# point:        [x, y, z]

# line_vec:     [x, y, z, nx, ny, nz, l]
# line_pp:      [x1, y1, z1, x2, y2, z2]

# plane:        [x, y, z, nx, ny, nz]

# box:          [x1, y1, z1, x2, y2, z2]
# box_rot:      [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]

# cylinder_vec: [x, y, z, nx, ny, nz, height, radius]
# cylinder_pp:  [x1, y1, z1, x2, y2, z2, radius]

# sphere:       [x, y, z, radius]
    */

    // Assert that the name is not already registered
    assert( 
        geometric_primitives_map_.find(name) == geometric_primitives_map_.end() 
    );


    GeometricPrimitive* primitive;



    if (type.compare("point") == 0)
    {
        primitive = new GeometricPoint(this, name, frame_id, visible, color, 
                                       parameters);


        
    }/*
    else if (type.compare("line_segment_pp") == 0)
    {
        primitive = new GeometricLineSegment();


        
    }
    else if (type.compare("line_segment_vec") == 0)
    {
        primitive = new GeometricLineSegment();


        
    }*/
    else if (type.compare("plane") == 0)
    {
        primitive = new GeometricPlane(this, name, frame_id, visible, color, 
                                       parameters);


        
    }/*
    else if (type.compare("box") == 0)
    {
        primitive = new GeometricBox();


        
    }
    else if (type.compare("box_rot") == 0)
    {
        primitive = new GeometricBox();


        
    }
    else if (type.compare("cylinder_vec") == 0)
    {
        primitive = new GeometricCylinder();


        
    }
    else if (type.compare("cylinder_pp") == 0)
    {
        primitive = new GeometricCylinder();



    }
    else if (type.compare("sphere") == 0)
    {
        primitive = new GeometricSphere();



    }*/
    else
    {

    }


    geometric_primitives_map_.insert( GeometricPrimitiveMapElement
        (
            name,
            primitive
        )
    );

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


