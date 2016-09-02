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
 * \file   task_factory.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/task_factory.h>

#include <hiqp/tasks/task_geometric_projection.h>
#include <hiqp/tasks/task_geometric_alignment.h>
#include <hiqp/tasks/task_jnt_config.h>
#include <hiqp/tasks/task_jnt_limits.h>
#include <hiqp/tasks/dynamics_first_order.h>
#include <hiqp/tasks/dynamics_jnt_limits.h>
#include <hiqp/tasks/dynamics_minimal_jerk.h>






namespace hiqp {



void TaskFactory::init
(
    GeometricPrimitiveMap* geometric_primitive_map,
    Visualizer* visualizer,
    unsigned int num_controls
)
{ 
    geometric_primitive_map_ = geometric_primitive_map; 
    visualizer_ = visualizer;
    num_controls_ = num_controls;
}






int TaskFactory::buildTask
(
	const std::string& name,
	std::size_t id,
    const std::string& type,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters,
    const std::vector<std::string>& behaviour_parameters,
    const std::chrono::steady_clock::time_point& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel,
    TaskDynamics*& dynamics,
    TaskFunction*& function
)
{
	dynamics = this->constructTaskDynamics(behaviour_parameters);
    if (dynamics == nullptr)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', could not parse the dynamics parameters! No task was added!");
        return -1;
    }

    function = this->constructTaskFunction(type, parameters);
    if (function == nullptr)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', could not parse the task parameters! No task was added!");
        delete dynamics;
        return -2;
    }

    function->setVisualizer(visualizer_);
	function->setGeometricPrimitiveMap(geometric_primitive_map_);
	function->setTaskName(name);
	function->setId(id);
	function->setPriority(priority);
	function->setVisibility(visibility);
    function->setTaskDynamics(dynamics);
	function->init(sampling_time, parameters, kdl_tree, num_controls_);
    function->computeInitialState(sampling_time, kdl_tree, kdl_joint_pos_vel);


    std::vector<std::string> default_beh_params = {"DynamicsFirstOrder", "1.0"};

    // There are some special cases for behaviour parameters
    std::vector<std::string> beh_params;
    if (behaviour_parameters.size() == 0) 
    {
        beh_params = default_beh_params;
    }
    else if (type.compare("TaskJntLimits") == 0)
    {
        beh_params.push_back(behaviour_parameters.at(0));
        beh_params.push_back(parameters.at(1)); // dq_max
    }

    dynamics->init(
        sampling_time, 
        beh_params, 
        function->getInitialState(),
        function->getFinalState(kdl_tree)
    );

    bool size_test1 = (function->e_.rows() != function->J_.rows());
    bool size_test2 = (function->e_.rows() != function->e_dot_star_.rows());
    bool size_test3 = (function->e_.rows() != function->task_types_.size());

    if (size_test1 || size_test2 || size_test3)
    {
        printHiqpWarning("While trying to add task '" + name 
            + "', the task dimensions was not properly setup! The task was not added!");
        delete function;
        delete dynamics;
        return -3;
    }


    return 0;
}







TaskDynamics* TaskFactory::constructTaskDynamics
(
    const std::vector<std::string>& parameters
)
{
    TaskDynamics* dynamics = nullptr;

    int size = parameters.size();
    if (size == 0 || (size == 1 && parameters.at(0).compare("NA") == 0) )
    {
        dynamics = new DynamicsFirstOrder();
    }

    else if (parameters.at(0).compare("DynamicsFirstOrder") == 0)
    {
        if (size == 2)
        {
            dynamics = new DynamicsFirstOrder();
        }
        else
        {
            printHiqpWarning("DynamicsFirstOrder requires 2 parameters, got " 
                + std::to_string(size) + "!");
        }
    }

    else if (parameters.at(0).compare("DynamicsJntLimits") == 0)
    {
        if (size == 1)
        {
            dynamics = new DynamicsJntLimits();
        }
        else
        {
            printHiqpWarning("DynamicsJntLimits requires "
                + std::to_string(1) 
                + " parameters, got " 
                + std::to_string(size) + "!");
        }
    }

    else if (parameters.at(0).compare("DynamicsMinimalJerk") == 0)
    {
        if (size == 2)
        {
            dynamics = new DynamicsMinimalJerk();
        }
        else
        {
            printHiqpWarning("DynamicsMinimalJerk requires "
                + std::to_string(2) 
                + " parameters, got " 
                + std::to_string(size) + "!");
        }
    }

    return dynamics;
}






TaskFunction* TaskFactory::constructTaskFunction
(
    const std::string& type,
    const std::vector<std::string>& parameters
)
{
    TaskFunction* function = nullptr;

    if (type.compare("TaskJntConfig") == 0)
    {
        function = new TaskJntConfig();
    }

    else if (type.compare("TaskJntLimits") == 0)
    {
        function = new TaskJntLimits();
    }

    else if (type.compare("TaskGeometricProjection") == 0)
    {
        std::string type1 = parameters.at(0);
        std::string type2 = parameters.at(1);



        if (type1.compare("point") == 0 && 
            type2.compare("point") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricPoint>();
        }
        else if (type1.compare("point") == 0 && 
                 type2.compare("line") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricLine>();
        }
        else if (type1.compare("point") == 0 && 
                 type2.compare("plane") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricPlane>();
        }
        else if (type1.compare("point") == 0 && 
                 type2.compare("box") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricBox>();
        }
        else if (type1.compare("point") == 0 && 
                 type2.compare("cylinder") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricCylinder>();
        }
        else if (type1.compare("point") == 0 && 
                 type2.compare("sphere") == 0)
        {
            function = new TaskGeometricProjection<GeometricPoint, GeometricSphere>();
        }


        else if (type1.compare("sphere") == 0 && 
                 type2.compare("plane") == 0)
        {
            function = new TaskGeometricProjection<GeometricSphere, GeometricPlane>();
        }
        else if (type1.compare("sphere") == 0 && 
                 type2.compare("sphere") == 0)
        {
            function = new TaskGeometricProjection<GeometricSphere, GeometricSphere>();
        }



        else
        {
            printHiqpWarning("TaskGeometricProjection does not allow primitive types: '"
                + type1 + "' and '" + type2 + "'!");
        }



    }

    else if (type.compare("TaskGeometricAlignment") == 0)
    {
        std::string type1 = parameters.at(0);
        std::string type2 = parameters.at(1);
        if (type1.compare("line") == 0 && type2.compare("line") == 0)
        {
            function = new TaskGeometricAlignment<GeometricLine, GeometricLine>();
        }
        else if (type1.compare("line") == 0 && type2.compare("plane") == 0)
        {
            function = new TaskGeometricAlignment<GeometricLine, GeometricPlane>();
        }
        else if (type1.compare("line") == 0 && type2.compare("cylinder") == 0)
        {
            function = new TaskGeometricAlignment<GeometricLine, GeometricCylinder>();
        }
        else if (type1.compare("line") == 0 && type2.compare("sphere") == 0)
        {
            function = new TaskGeometricAlignment<GeometricLine, GeometricSphere>();
        }
        else
        {
            printHiqpWarning("TaskGeometricAlignment does not allow primitive types: '"
                + type1 + "' and '" + type2 + "'!");
        }
    }

    else
    {
        printHiqpWarning("Task type name '" + type + "' was not recognized!");
    }

    return function;
}









} // namespace hiqp