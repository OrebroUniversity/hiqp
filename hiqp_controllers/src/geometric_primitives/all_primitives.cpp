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
 * \file   geometric_primitive_map.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_plane.h>

// STL Includes
//#include <iostream>
#include <cassert>





namespace hiqp {




GeometricPrimitive::GeometricPrimitive
(
	const std::string& name,
	const std::string& frame_id,
	bool visible,
	const std::vector<double>& color
)
: name_(name), frame_id_(frame_id), visible_(visible)
{
	assert(color.size() == 4);
	r_ = color.at(0);
	g_ = color.at(1);
	b_ = color.at(2);
	a_ = color.at(3);
}








} // namespace hiqp