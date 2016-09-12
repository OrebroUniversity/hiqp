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
 * \file   ros_topic_subscriber.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


// HiQP Includes
#include <hiqp/ros_topic_subscriber.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>

#include <hiqp_msgs_srvs/Vector3d.h>

// STL Includes
#include <iostream>

// Wintracker Includes
#include <geometry_msgs/PoseStamped.h>




namespace hiqp
{





template<>
void ROSTopicSubscriber::topicCallback<geometry_msgs::PoseStamped>
(
	const geometry_msgs::PoseStamped& msg
)
{
	std::vector<double> point_params;
	point_params.push_back(msg.pose.position.x);
	point_params.push_back(msg.pose.position.y);
	point_params.push_back(msg.pose.position.z);
	primitive_map_->updateGeometricPrimitive<GeometricPoint>("teleop_point", point_params);

	point_params.push_back(0.05);
	primitive_map_->updateGeometricPrimitive<GeometricSphere>("teleop_sphere", point_params);

	/*
	std::cout << "pos = (" 
		<< msg.pose.position.x << ", "
		<< msg.pose.position.y << ", "
		<< msg.pose.position.z << ")\n"
		<< "rot = ("
		<< msg.pose.orientation.x << ", "
		<< msg.pose.orientation.y << ", "
		<< msg.pose.orientation.z << ", "
		<< msg.pose.orientation.w << ")\n\n";
		*/
}






template<>
void ROSTopicSubscriber::topicCallback<hiqp_msgs_srvs::Vector3d>
(
	const hiqp_msgs_srvs::Vector3d& msg
)
{
	GeometricCylinder* cylinder = primitive_map_->getGeometricPrimitive<GeometricCylinder>("thecylinder");

	if (cylinder == nullptr)
	{
		printHiqpWarning("Vector3d callback : couldn't find primitive 'thecylinder'");
	}

	std::vector<double> params;
	params.push_back(cylinder->getDirectionX());
	params.push_back(cylinder->getDirectionY());
	params.push_back(cylinder->getDirectionZ());
	params.push_back(cylinder->getOffsetX() + msg.val1);
	params.push_back(cylinder->getOffsetY() + msg.val2);
	params.push_back(cylinder->getOffsetZ());
	params.push_back(cylinder->getRadius() + msg.val3);
	params.push_back(cylinder->getHeight());
	primitive_map_->updateGeometricPrimitive<GeometricCylinder>("thecylinder", params);

	std::cout << "Updated thecylinder\n";

}





}




