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
 * \file   ros_visualizer.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <iostream>
#include <cmath>
 
#include <hiqp/ros_visualizer.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>






namespace hiqp
{



ROSVisualizer::ROSVisualizer()
: next_id_(0)
{}





int ROSVisualizer::init
(
	ros::NodeHandle* controller_nh
)
{
	controller_nh_ = controller_nh;

	marker_pub_ = controller_nh_->advertise<visualization_msgs::Marker>
		("visualization_marker", 1);
}





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								 A P P L Y                                    //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

int ROSVisualizer::apply
(
	int id,
	GeometricPoint* point,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + point->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = action; 

    marker.pose.position.x = point->getX();
    marker.pose.position.y = point->getY();
    marker.pose.position.z = point->getZ();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2*kPointRadius;
    marker.scale.y = 2*kPointRadius;
    marker.scale.z = 2*kPointRadius;

    marker.color.r = point->getRedComponent();
    marker.color.g = point->getGreenComponent();
    marker.color.b = point->getBlueComponent();
    marker.color.a = point->getAlphaComponent();

    marker.lifetime = ros::Duration(0);

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





int ROSVisualizer::apply
(
	int id,
	GeometricLine* line,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + line->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = action; 

    double length = kInfiniteLength;

    double v1 = line->getDirectionX();
    double v2 = line->getDirectionY();
    double v3 = line->getDirectionZ();

    double p1 = line->getOffsetX() + v1 * length/2;
    double p2 = line->getOffsetY() + v2 * length/2;
    double p3 = line->getOffsetZ() + v3 * length/2;

    Eigen::Vector3d v;
    v << v1, v2, v3;

    // Quaternion that aligns the z-axis with the line direction
    Eigen::Quaterniond q;
    q.setFromTwoVectors(Eigen::Vector3d::UnitZ(), v);

    marker.pose.position.x = p1;
    marker.pose.position.y = p2;
    marker.pose.position.z = p3;

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    
    marker.scale.x = 2*kLineRadius;
    marker.scale.y = 2*kLineRadius;
    marker.scale.z = length;

    marker.color.r = line->getRedComponent();
    marker.color.g = line->getGreenComponent();
    marker.color.b = line->getBlueComponent();
    marker.color.a = line->getAlphaComponent();

    marker.lifetime = ros::Duration(0);

    // std::cout << "Marker line: x = " << marker.pose.position.x << "\n"
    //           << "             y = " << marker.pose.position.y << "\n"
    //           << "             z = " << marker.pose.position.z << "\n"
    //           << "         rot x = " << marker.pose.orientation.x << "\n"
    //           << "         rot y = " << marker.pose.orientation.y << "\n"
    //           << "         rot z = " << marker.pose.orientation.z << "\n"
    //           << "         rot w = " << marker.pose.orientation.w << "\n"
    //           << "         sca x = " << marker.scale.x << "\n"
    //           << "         sca y = " << marker.scale.y << "\n"
    //           << "         sca z = " << marker.scale.z << "\n";

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





int ROSVisualizer::apply
(
	int id,
	GeometricPlane* plane,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + plane->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action; 

    marker.pose.position.x = plane->getOffset();
    marker.pose.position.y = plane->getOffset();
    marker.pose.position.z = plane->getOffset();

    marker.pose.orientation.x = plane->getNormalX();
    marker.pose.orientation.y = plane->getNormalY();
    marker.pose.orientation.z = plane->getNormalZ();
    marker.pose.orientation.w = 1.0;

    marker.scale.x = kInfiniteLength/2;
    marker.scale.y = kInfiniteLength/2;
    marker.scale.z = kPlaneThickness;

    marker.color.r = plane->getRedComponent();
    marker.color.g = plane->getGreenComponent();
    marker.color.b = plane->getBlueComponent();
    marker.color.a = plane->getAlphaComponent();

    marker.lifetime = ros::Duration(0); // forever

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





int ROSVisualizer::apply
(
	int id,
	GeometricBox* box,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + box->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = action; 

    marker.pose.position.x = box->getCenterX();
    marker.pose.position.y = box->getCenterY();
    marker.pose.position.z = box->getCenterZ();

    marker.pose.orientation.x = box->getRotationX();
    marker.pose.orientation.y = box->getRotationY();
    marker.pose.orientation.z = box->getRotationZ();
    marker.pose.orientation.w = box->getRotationAngle();

    marker.scale.x = box->getDimX();
    marker.scale.y = box->getDimY();
    marker.scale.z = box->getDimZ();

    marker.color.r = box->getRedComponent();
    marker.color.g = box->getGreenComponent();
    marker.color.b = box->getBlueComponent();
    marker.color.a = box->getAlphaComponent();

    marker.lifetime = ros::Duration(0); // forever

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





int ROSVisualizer::apply
(
	int id,
	GeometricCylinder* cylinder,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + cylinder->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = action; 

    marker.pose.position.x = cylinder->getOffsetX();
    marker.pose.position.y = cylinder->getOffsetY();
    marker.pose.position.z = cylinder->getOffsetZ();
    if (cylinder->isInfinite())  
    {
    	marker.pose.position.x -= cylinder->getDirectionX() * kInfiniteLength/2;
    	marker.pose.position.y -= cylinder->getDirectionY() * kInfiniteLength/2;
    	marker.pose.position.z -= cylinder->getDirectionZ() * kInfiniteLength/2;
	}

    marker.pose.orientation.x = cylinder->getDirectionX();
    marker.pose.orientation.y = cylinder->getDirectionY();
    marker.pose.orientation.z = cylinder->getDirectionZ();
    marker.pose.orientation.w = 1.0;

    if (cylinder->isInfinite())  marker.scale.x = kInfiniteLength;
    else                     marker.scale.x = cylinder->getHeight();
    marker.scale.y = 2*cylinder->getRadius();
    marker.scale.z = 2*cylinder->getRadius();

    marker.color.r = cylinder->getRedComponent();
    marker.color.g = cylinder->getGreenComponent();
    marker.color.b = cylinder->getBlueComponent();
    marker.color.a = cylinder->getAlphaComponent();

    marker.lifetime = ros::Duration(0);

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





int ROSVisualizer::apply
(
	int id,
	GeometricSphere* sphere,
	int action
)
{
	visualization_msgs::Marker marker;

	marker.header.frame_id = "/" + sphere->getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = kNamespace;
    if (action == visualization_msgs::Marker::ADD)  marker.id = next_id_;
    else                                            marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = action; 

    marker.pose.position.x = sphere->getX();
    marker.pose.position.y = sphere->getY();
    marker.pose.position.z = sphere->getZ();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 2*sphere->getRadius();
    marker.scale.y = 2*sphere->getRadius();
    marker.scale.z = 2*sphere->getRadius();

    marker.color.r = sphere->getRedComponent();
    marker.color.g = sphere->getGreenComponent();
    marker.color.b = sphere->getBlueComponent();
    marker.color.a = sphere->getAlphaComponent();

    marker.lifetime = ros::Duration(0);

    marker_pub_.publish(marker);

    if (action == visualization_msgs::Marker::ADD)
    {
    	next_id_++;
    	return next_id_-1;
    }
    else
    {
    	return id;
    }
}





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								   A D D                                      //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

int ROSVisualizer::add
(
    GeometricPoint* point
)
{
	return apply(0, point, visualization_msgs::Marker::ADD);
}





int ROSVisualizer::add
(
    GeometricLine* line
)
{
	return apply(0, line, visualization_msgs::Marker::ADD);
}





int ROSVisualizer::add
(
    GeometricPlane* plane
)
{
	return apply(0, plane, visualization_msgs::Marker::ADD);
}





int ROSVisualizer::add
(
    GeometricBox* box
)
{
	return apply(0, box, visualization_msgs::Marker::ADD);
}





int ROSVisualizer::add
(
    GeometricCylinder* cylinder
)
{
	return apply(0, cylinder, visualization_msgs::Marker::ADD);
}





int ROSVisualizer::add
(
    GeometricSphere* sphere
)
{
	return apply(0, sphere, visualization_msgs::Marker::ADD);
}





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								U P D A T E                                   //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

void ROSVisualizer::update
(
	int id, 
	GeometricPoint* point
)
{
	apply(id, point, visualization_msgs::Marker::MODIFY);
}





void ROSVisualizer::update
(
	int id, 
	GeometricLine* line
)
{
	apply(id, line, visualization_msgs::Marker::MODIFY);
}





void ROSVisualizer::update
(
	int id, 
	GeometricPlane* plane
)
{
	apply(id, plane, visualization_msgs::Marker::MODIFY);
}





void ROSVisualizer::update
(
	int id, 
	GeometricBox* box
)
{
	apply(id, box, visualization_msgs::Marker::MODIFY);
}





void ROSVisualizer::update
(
	int id, 
	GeometricCylinder* cylinder
)
{
	apply(id, cylinder, visualization_msgs::Marker::MODIFY);
}





void ROSVisualizer::update
(
	int id, 
	GeometricSphere* sphere
)
{
	apply(id, sphere, visualization_msgs::Marker::MODIFY);
}





////////////////////////////////////////////////////////////////////////////////
// 																			  //
//								R E M O V E                                   //
// 																			  //
////////////////////////////////////////////////////////////////////////////////

void ROSVisualizer::remove
(
	int id
)
{

}











} // namespace hiqp