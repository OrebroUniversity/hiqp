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
 * \file   task_geometric_alignment.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
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

// Orocos KDL Includes
//#include <kdl/treefksolverpos_recursive.hpp>
//#include <kdl/treejnttojacsolver.hpp>

#include <iostream>
#include <cmath>
//#include <string>
//#include <sstream>






namespace hiqp
{




template<>
int TaskGeometricAlignment<GeometricLine, GeometricLine>::align
(
	GeometricLine* line1,
	GeometricLine* line2
)
{
	KDL::Vector v1 = pose_a_.M * line1->getDirectionKDL();
	KDL::Vector v2 = pose_b_.M * line2->getDirectionKDL();

	double d = KDL::dot(v1, v2);

	e_(0) = d - std::cos(delta_);

	std::cout << "e = " << e_(0) << "\n";

	KDL::Vector v = v1 * v2;    // v = v1 x v2

	for (int q_nr = 0; q_nr < jacobian_a_.columns(); ++q_nr)
	{
		KDL::Vector Ja = jacobian_a_.getColumn(q_nr).rot;
		KDL::Vector Jb = jacobian_b_.getColumn(q_nr).rot;

		J_(0, q_nr) = KDL::dot( v, (Ja - Jb) );
	}

	return 0;
}




} // namespace hiqp