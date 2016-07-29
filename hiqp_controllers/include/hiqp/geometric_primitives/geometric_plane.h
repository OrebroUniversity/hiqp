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
 * \file   geometric_plane.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_PLANE_H
#define HIQP_GEOMETRIC_PLANE_H




#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricPlane
 * \brief 
 */  
class GeometricPlane : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	GeometricPlane
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color,
		const std::vector<std::string>& parameters
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{
		assert(parameters.size() == 4);
		n_(0) = std::stod( parameters.at(0) );
		n_(1) = std::stod( parameters.at(1) );
		n_(2) = std::stod( parameters.at(2) );
		d_ = std::stod( parameters.at(3) );
	}



	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~GeometricPlane() noexcept {}



	inline const KDL::Vector& getNormal()
	{ return n_; }

	inline double getOffset()
	{ return d_; }



	void draw(TaskVisualizer* visualizer)
	{
		if (visible_)
		{
			visualizer->createPlane(frame_id_, n_(0), n_(1), n_(2), d_, 
				                    r_, g_, b_, a_);
		}
	}




private:

	// No copying of this class is allowed !
	GeometricPlane(const GeometricPlane& other) = delete;
	GeometricPlane(GeometricPlane&& other) = delete;
	GeometricPlane& operator=(const GeometricPlane& other) = delete;
	GeometricPlane& operator=(GeometricPlane&& other) noexcept = delete;

	KDL::Vector   n_; // the normal vector or the plane
	double 		  d_;






};











} // namespace hiqp

#endif // include guard