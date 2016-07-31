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
 * \file   geometric_sphere.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_SPHERE_H
#define HIQP_GEOMETRIC_SPHERE_H



#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricSphere
 * \brief 
 */  
class GeometricSphere : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricSphere
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
		p_(0) = std::stod( parameters.at(0) );
		p_(1) = std::stod( parameters.at(1) );
		p_(2) = std::stod( parameters.at(2) );
		radius_ = std::stod( parameters.at(3) );
	}



	/*!
     * \brief Destructor
     */
	~GeometricSphere() noexcept {}



	inline const KDL::Vector& getPoint()
	{ return p_; }

	inline double getRadius()
	{ return radius_; }

	inline double getX() { return p_(0); }
	inline double getY() { return p_(1); }
	inline double getZ() { return p_(2); }




protected:

	KDL::Vector    p_; // the offset of the sphere

	double         radius_; // the radius of the sphere





private:

	// No copying of this class is allowed !
	GeometricSphere(const GeometricSphere& other) = delete;
	GeometricSphere(GeometricSphere&& other) = delete;
	GeometricSphere& operator=(const GeometricSphere& other) = delete;
	GeometricSphere& operator=(GeometricSphere&& other) noexcept = delete;

};











} // namespace hiqp

#endif // include guard