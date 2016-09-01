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
 * \file   geometric_point.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   August, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/geometric_primitives/geometric_point.h>

namespace hiqp
{



GeometricPrimitive::GeometricPrimitive
(
	const std::string& name,
	const std::string& frame_id,
	bool visible,
	const std::vector<double>& color
)
: name_(name), frame_id_(frame_id), visible_(visible)
{
	//assert(color.size() == 4);
	r_ = color.at(0);
	g_ = color.at(1);
	b_ = color.at(2);
	a_ = color.at(3);
}



GeometricPoint::GeometricPoint
(
	const std::string& name,
	const std::string& frame_id,
	bool visible,
	const std::vector<double>& color
)
: GeometricPrimitive(name, frame_id, visible, color)
{}




int GeometricPoint::init(const std::vector<double>& parameters)
{
	int size = parameters.size();
	if (size != 3)
	{
		printHiqpWarning("GeometricPoint requires 3 parameters, got " 
			+ std::to_string(size) + "! Initialization failed!");
		return -1;
	}

	std::cout << "point init: " 
		<< parameters.at(0) << ", " 
		<< parameters.at(1) << ", " 
		<< parameters.at(2) << "\n";

	kdl_p_(0) = parameters.at(0);
	kdl_p_(1) = parameters.at(1);
	kdl_p_(2) = parameters.at(2);

	eigen_p_ << kdl_p_(0), kdl_p_(1), kdl_p_(2);

	return 0;
}




} // namespace hiqp