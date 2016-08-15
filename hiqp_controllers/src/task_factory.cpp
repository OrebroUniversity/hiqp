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






TaskFunction* TaskFactory::buildTaskFunction
(
	const std::string& name,
	std::size_t id,
    const std::string& type,
    unsigned int priority,
    bool visibility,
    const std::vector<std::string>& parameters,
    TaskDynamics* dynamics
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
        if (type1.compare("line") == 0 && 
            type2.compare("line") == 0)
        {
            function = new TaskGeometricAlignment<GeometricLine, GeometricLine>();
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




    if (function != nullptr)
    {
    	function->setVisualizer(visualizer_);
    	function->setGeometricPrimitiveMap(geometric_primitive_map_);

    	function->setTaskName(name);
    	function->setId(id);
    	function->setPriority(priority);
    	function->setVisibility(visibility);
        function->setTaskDynamics(dynamics);

    	function->init(parameters, num_controls_);
    }

    return function;
}






TaskDynamics* TaskFactory::buildTaskDynamics
(
	const std::vector<std::string>& parameters
)
{
    TaskDynamics* dynamics = nullptr;

    int size = parameters.size();
	if (size == 0 || (size == 1 && parameters.at(0).compare("NA") == 0) )
    {
        dynamics = new DynamicsFirstOrder();
        dynamics->init( {"DynamicsFirstOrder", "1.0"} );
    }

	if (parameters.at(0).compare("DynamicsFirstOrder") == 0)
    {
        if (size == 2)
        {
            dynamics = new DynamicsFirstOrder();
            dynamics->init(parameters);
        }
        else
        {
            printHiqpWarning("DynamicsFirstOrder requires 2 parameters, got " 
                + std::to_string(size) + "!");
        }
    }

    else if (parameters.at(0).compare("DynamicsJntLimits") == 0)
    {
        if (size == num_controls_ + 1)
        {
            dynamics = new DynamicsJntLimits();
            dynamics->init(parameters);
        }
        else
        {
            printHiqpWarning("DynamicsJntLimits requires "
                + std::to_string(num_controls_ + 1) 
                + " parameters, got " 
                + std::to_string(size) + "!");
        }
    }

    return dynamics;
}



} // namespace hiqp