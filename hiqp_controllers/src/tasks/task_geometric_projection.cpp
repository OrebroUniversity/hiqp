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
 * \file   task_geometric_projection.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/tasks/task_geometric_projection.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
//#include <kdl/treefksolverpos_recursive.hpp>
//#include <kdl/treejnttojacsolver.hpp>

#include <iostream>
#include <string>
#include <sstream>






namespace hiqp
{







template<>
int TaskGeometricProjection<GeometricPoint, GeometricPoint>::project
(
	GeometricPoint* point1, 
	GeometricPoint* point2
)
{
	std::cout << "\nproject()\n";

	KDL::Vector p1__ = pose_a_.M * point1->getPointKDL();

	KDL::Vector p1( pose_a_.p.x() + p1__(0), 
		            pose_a_.p.y() + p1__(1), 
		            pose_a_.p.z() + p1__(2) );

	KDL::Vector p2__ = pose_b_.M * point2->getPointKDL();

	KDL::Vector p2( pose_b_.p.x() + p2__(0), 
		            pose_b_.p.y() + p2__(1), 
		            pose_b_.p.z() + p2__(2) );

	KDL::Vector d = p2 - p1;

	e_(0) = KDL::dot(d, d);

	std::cout << "p1 = " << p1 << "\n";
	std::cout << "p2 = " << p2 << "\n";
	std::cout << "d = " << d << "\n";
	std::cout << "e_ = " << e_(0) << "\n";

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   jacobian_a_.getColumn(q_nr).vel.x()
		              + jacobian_a_.getColumn(q_nr).vel.y()
		              + jacobian_a_.getColumn(q_nr).vel.z();
	}
	
	return 0;

}





template<>
int TaskGeometricProjection<GeometricPoint, GeometricLine>::project
(
	GeometricPoint* point, 
	GeometricLine* line
)
{

	KDL::Vector p__ = pose_a_.M * point->getPointKDL();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	KDL::Vector n = pose_b_.M * line->getDirectionKDL();
	KDL::Vector d = pose_b_.M * line->getOffsetKDL();

	KDL::Vector p_tilde = p - d;

	KDL::Vector l = p_tilde - KDL::dot(p_tilde, n) * n;

	e_(0) = KDL::dot(l, l);

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   jacobian_a_.getColumn(q_nr).vel.x()
		              + jacobian_a_.getColumn(q_nr).vel.y()
		              + jacobian_a_.getColumn(q_nr).vel.z();
	}
	
	return 0;

}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricPlane>::project
(
	GeometricPoint* point, 
	GeometricPlane* plane
)
{

	KDL::Vector p__ = pose_a_.M * point->getPointKDL();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	KDL::Vector n = pose_b_.M * plane->getNormalKDL();

	double d = plane->getOffset() + KDL::dot(n, pose_b_.p);

	e_(0) = KDL::dot(n, p) - d;

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   n(0) * jacobian_a_.getColumn(q_nr).vel.x()
		              + n(1) * jacobian_a_.getColumn(q_nr).vel.y()
		              + n(2) * jacobian_a_.getColumn(q_nr).vel.z();
	}
	
	return 0;

}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricBox>::project
(
	GeometricPoint* point, 
	GeometricBox* box
)
{
/*
	KDL::Vector p__ = pose_a_.M * point->getPoint();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	KDL::Vector n = pose_b_.M * box->getNormal();
	KDL::Vector d = pose_b_.M * box->getOffset();

	KDL::Vector p_tilde = p - d;


	e_(0) = 0;

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   jacobian_a_.getColumn(q_nr).vel.x()
		              + jacobian_a_.getColumn(q_nr).vel.y()
		              + jacobian_a_.getColumn(q_nr).vel.z();
	}
	*/
	return 0;

}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricCylinder>::project
(
	GeometricPoint* point, 
	GeometricCylinder* cylinder
)
{
/*
	KDL::Vector p__ = pose_a_.M * point->getPoint();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	//KDL::Vector n = pose_b_.M * cylinder->getNormal();
	KDL::Vector d = pose_b_.M * cylinder->getOffset();

	KDL::Vector p_tilde = p - d;


	e_(0) = 0;

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   jacobian_a_.getColumn(q_nr).vel.x()
		              + jacobian_a_.getColumn(q_nr).vel.y()
		              + jacobian_a_.getColumn(q_nr).vel.z();
	}
	*/
	return 0;

}







template<>
int TaskGeometricProjection<GeometricPoint, GeometricSphere>::project
(
	GeometricPoint* point, 
	GeometricSphere* sphere
)
{
/*
	KDL::Vector p__ = pose_a_.M * point->getPoint();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	KDL::Vector d = pose_b_.M * sphere->getOffset();

	e_(0) = p - d - sphere->getRadius();

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   jacobian_a_.getColumn(q_nr).vel.x()
		              + jacobian_a_.getColumn(q_nr).vel.y()
		              + jacobian_a_.getColumn(q_nr).vel.z();
	}
	*/
	return 0;

}







} // namespace hiqp