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

/*
 * \file   task_geometric_alignment.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/tasks/task_geometric_alignment.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <hiqp/hiqp_utils.h>

#include <iostream>
#include <cmath>





namespace hiqp
{
namespace tasks
{





template<>
int TaskGeometricAlignment<GeometricLine, GeometricLine>::align
(
	std::shared_ptr<GeometricLine> line1,
	std::shared_ptr<GeometricLine> line2
)
{
	KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
	KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricPlane>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricPlane> plane
)
{
	KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();
	KDL::Vector v2 = - ( pose_b_.M * plane->getNormalKDL() );

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricCylinder>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricCylinder> cylinder
)
{
	KDL::Vector v1 = - (pose_a_.M * line->getDirectionKDL());

	KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL();
	KDL::Vector d = pose_b_.p + pose_b_.M * cylinder->getOffsetKDL();
	KDL::Vector v = pose_b_.M * cylinder->getDirectionKDL();

	KDL::Vector x = KDL::dot( (p-d), v) * v;

	KDL::Vector v2 = (p-d) - x;

	v2.Normalize();

	return alignVectors(v1, v2);
}





template<>
int TaskGeometricAlignment<GeometricLine, GeometricSphere>::align
(
	std::shared_ptr<GeometricLine> line,
	std::shared_ptr<GeometricSphere> sphere
)
{
	KDL::Vector v1 = pose_a_.M * line->getDirectionKDL();

	KDL::Vector p = pose_a_.p + pose_a_.M * line->getOffsetKDL();
	KDL::Vector d = pose_b_.p + pose_b_.M * sphere->getCenterKDL();

	KDL::Vector v2 = d - p;

	v2.Normalize();

	return alignVectors(v1, v2);
}





} // namespace tasks

} // namespace hiqp












