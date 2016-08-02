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
 * \file   geometric_cylinder.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_CYLINDER_H
#define HIQP_GEOMETRIC_CYLINDER_H




#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricCylinder
 * \brief 
 *
 * The parameter interface for this primitive is:
 * cylinder:     [nx, ny, nz,  x,  y,  z, radius, height]
 */  
class GeometricCylinder : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricCylinder
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color,
		const std::vector<std::string>& parameters
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{
		int size = parameters.size();
		assert(size == 8);

		/*
		if (size == 7)
		{
			v_(0) =   std::stod( parameters.at(3) )
			        - std::stod( parameters.at(0) );
			v_(1) =   std::stod( parameters.at(4) )
			        - std::stod( parameters.at(1) );
			v_(2) =   std::stod( parameters.at(5) )
			        - std::stod( parameters.at(2) );

			p_(0) = std::stod( parameters.at(0) );
			p_(1) = std::stod( parameters.at(1) );
			p_(2) = std::stod( parameters.at(2) );

			h_ = v_.Norm();
			v_.Normalize();

			radius_ = std::stod( parameters.at(6) );
		}
		else 
		{
			*/
			v_(0) = std::stod( parameters.at(0) );
			v_(1) = std::stod( parameters.at(1) );
			v_(2) = std::stod( parameters.at(2) );
			v_.Normalize();

			p_(0) = std::stod( parameters.at(3) );
			p_(1) = std::stod( parameters.at(4) );
			p_(2) = std::stod( parameters.at(5) );

			radius_ = std::stod( parameters.at(6) );

			std::string height = parameters.at(7);
			if (height.compare("INF")==0 || 
				height.compare("Inf")==0 || 
				height.compare("inf")==0)
			{
				h_ = -1;
			}
			else
			{
				h_ = std::stod( height );
			}

			
		//}

		
	}



	/*!
     * \brief Destructor
     */
	~GeometricCylinder() noexcept {}

	inline const KDL::Vector& getDirection()
	{ return v_; }

	inline const KDL::Vector& getOffset()
	{ return p_; }

	inline double getHeight()
	{ return h_; }

	inline double getRadius()
	{ return radius_; }

	inline bool isInfinite()
	{ return (h_ < 0); }

	inline double getDirectionX() { return v_(0); }
	inline double getDirectionY() { return v_(1); }
	inline double getDirectionZ() { return v_(2); }

	inline double getOffsetX() { return p_(0); }
	inline double getOffsetY() { return p_(1); }
	inline double getOffsetZ() { return p_(2); }




protected:

	KDL::Vector   v_; // the directional vector of the cylinder

	KDL::Vector   p_; // the offset of the cylinder base

	double        h_; // the height of the cylinder

	double        radius_; // the radius of the cylinder





private:

	// No copying of this class is allowed !
	GeometricCylinder(const GeometricCylinder& other) = delete;
	GeometricCylinder(GeometricCylinder&& other) = delete;
	GeometricCylinder& operator=(const GeometricCylinder& other) = delete;
	GeometricCylinder& operator=(GeometricCylinder&& other) noexcept = delete;

};











} // namespace hiqp

#endif // include guard