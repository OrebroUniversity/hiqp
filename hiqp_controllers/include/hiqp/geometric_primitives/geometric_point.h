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
 * \file   geometric_point.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_POINT_H
#define HIQP_GEOMETRIC_POINT_H



#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricPoint
 * \brief 
 */  
class GeometricPoint : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	GeometricPoint
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color,
		const std::vector<std::string>& parameters
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{
		assert(parameters.size() == 3);
		p_(0) = std::stod( parameters.at(0) );
		p_(1) = std::stod( parameters.at(1) );
		p_(2) = std::stod( parameters.at(2) );
	}



	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~GeometricPoint() noexcept {}



	inline const KDL::Vector& getPoint()
	{ return p_; }


	void draw(TaskVisualizer* visualizer)
	{
		if (visible_)
		{
			visualizer->createSphere(frame_id_, p_(0), p_(1), p_(2), 0.005, 
				                     r_, g_, b_, a_);
		}
	}



private:

	// No copying of this class is allowed !
	GeometricPoint(const GeometricPoint& other) = delete;
	GeometricPoint(GeometricPoint&& other) = delete;
	GeometricPoint& operator=(const GeometricPoint& other) = delete;
	GeometricPoint& operator=(GeometricPoint&& other) noexcept = delete;

	KDL::Vector    p_;






};











} // namespace hiqp

#endif // include guard