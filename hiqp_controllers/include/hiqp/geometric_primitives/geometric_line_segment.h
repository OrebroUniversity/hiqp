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
 * \file   geometric_line_segment.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_LINE_SEGMENT_H
#define HIQP_GEOMETRIC_LINE_SEGMENT_H




#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricLineSegment
 * \brief 
 * 
 * The parameter interface for this primitive is:
 * line:         [nx, ny, nz,  x,  y,  z, l]
 * line:         [x1, y1, z1, x2, y2, z2]
 */  
class GeometricLineSegment : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricLineSegment
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
		assert(size == 6 || size == 7);

		// parameters: ['1', '0', '0', '0', '0.1', '0', 'Inf']"
		if (size == 7)
		{
			v_(0) = std::stod( parameters.at(0) );
			v_(1) = std::stod( parameters.at(1) );
			v_(2) = std::stod( parameters.at(2) );
			
			v_.Normalize();
			
			p_(0) = std::stod( parameters.at(3) );
			p_(1) = std::stod( parameters.at(4) );
			p_(2) = std::stod( parameters.at(5) );

			std::string length = parameters.at(6);
			if (length.compare("INF")==0 || 
				length.compare("Inf")==0 || 
				length.compare("inf")==0)
			{
				l_ = -1;
			}
			else
			{
				l_ = std::stod( length );
			}
		}
		else
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

			l_ = v_.Norm();
			v_.Normalize();
		}

		std::cout << "v_ = " << v_(0) << ", " << v_(1) << ", " << v_(2) << "\n";
		std::cout << "p_ = " << p_(0) << ", " << p_(1) << ", " << p_(2) << "\n";
		std::cout << "l_ = " << l_ <<"\n";

		
	}



	/*!
     * \brief Destructor
     */
	~GeometricLineSegment() noexcept {}

	inline const KDL::Vector& getDirection()
	{ return v_; }

	inline const KDL::Vector& getOffset()
	{ return p_; }

	inline double getLength()
	{ return l_; }

	inline bool isInfinite()
	{ return (l_ < 0); }

	inline double getDirectionX() { return v_(0); }
	inline double getDirectionY() { return v_(1); }
	inline double getDirectionZ() { return v_(2); }

	inline double getOffsetX() { return p_(0); }
	inline double getOffsetY() { return p_(1); }
	inline double getOffsetZ() { return p_(2); }


protected:

	KDL::Vector   v_; // the directional vector of the line

	KDL::Vector   p_; // the offset

	double 		  l_; // the length of the line segment, 
	                  // negative if the line is infinite



private:

	// No copying of this class is allowed !
	GeometricLineSegment(const GeometricLineSegment& other) = delete;
	GeometricLineSegment(GeometricLineSegment&& other) = delete;
	GeometricLineSegment& operator=(const GeometricLineSegment& other) = delete;
	GeometricLineSegment& operator=(GeometricLineSegment&& other) noexcept = delete;


};











} // namespace hiqp

#endif // include guard