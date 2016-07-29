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



#include <hiqp/impl/task_geometric_projection.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_plane.h>

//#include <hiqp/hiqp_utils.h>

// Orocos KDL Includes
//#include <kdl/treefksolverpos_recursive.hpp>
//#include <kdl/treejnttojacsolver.hpp>

#include <iostream>
#include <string>
#include <sstream>






namespace hiqp
{







template<>
int TaskGeometricProjection<GeometricPoint, GeometricPlane>::project
(
	GeometricPoint* point, 
	GeometricPlane* plane
)
{

	std::cout << "inside project()\n";

	KDL::Vector p__ = pose_a_.M * point->getPoint();

	KDL::Vector p( pose_a_.p.x() + p__(0), 
		           pose_a_.p.y() + p__(1), 
		           pose_a_.p.z() + p__(2) );

	KDL::Vector n = pose_b_.M * plane->getNormal();

	double d = plane->getOffset() + KDL::dot(n, pose_b_.p);

	e_(0) = KDL::dot(n, p) - d;

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		J_(0, q_nr) =   n(0) * jacobian_a_.getColumn(q_nr).vel.x()
		              + n(1) * jacobian_a_.getColumn(q_nr).vel.y()
		              + n(2) * jacobian_a_.getColumn(q_nr).vel.z();
	}

	std::cout << "e = " << e_(0) << "\n";
	std::cout << "J = " << J_ << "\n";
	
	return 0;

}







} // namespace hiqp