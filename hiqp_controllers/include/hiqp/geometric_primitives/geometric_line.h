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
 * \file   geometric_line.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_LINE_H
#define HIQP_GEOMETRIC_LINE_H




#include <hiqp/geometric_primitives/geometric_primitive.h>
#include <hiqp/hiqp_utils.h>

#include <kdl/frames.hpp>

#include <Eigen/Dense>




namespace hiqp
{




/*!
 * \class GeometricLineSegment
 * \brief 
 * 
 * The parameter interface for this primitive is:
 * line:         [nx, ny, nz,  x,  y,  z]
 */  
class GeometricLine : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricLine
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{}

	/*!
     * \brief Destructor
     */
	~GeometricLine() noexcept {}



	int init(const std::vector<std::string>& parameters)
	{
		int size = parameters.size();
		assert(size == 6);

		kdl_v_(0) = std::stod( parameters.at(0) );
		kdl_v_(1) = std::stod( parameters.at(1) );
		kdl_v_(2) = std::stod( parameters.at(2) );
		
		kdl_v_.Normalize();
		
		kdl_p_(0) = std::stod( parameters.at(3) );
		kdl_p_(1) = std::stod( parameters.at(4) );
		kdl_p_(2) = std::stod( parameters.at(5) );

		l_ = -1; // infinitely long line

		eigen_v_ << kdl_v_(0), kdl_v_(1), kdl_v_(2);
		eigen_p_ << kdl_p_(0), kdl_p_(1), kdl_p_(2);

		return 0;
	}



	inline const KDL::Vector&      getDirectionKDL()    { return kdl_v_; }
	inline const Eigen::Vector3d&  getDirectionEigen()  { return eigen_v_; }

	inline const KDL::Vector&      getOffsetKDL()    { return kdl_p_; }
	inline const Eigen::Vector3d&  getOffsetEigen()  { return eigen_p_; }

	// inline double getLength() { return l_; }

	inline bool isInfinite()
	{ return (l_ < 0); }

	inline double getDirectionX() { return kdl_v_(0); }
	inline double getDirectionY() { return kdl_v_(1); }
	inline double getDirectionZ() { return kdl_v_(2); }

	inline double getOffsetX() { return kdl_p_(0); }
	inline double getOffsetY() { return kdl_p_(1); }
	inline double getOffsetZ() { return kdl_p_(2); }


protected:

	KDL::Vector      kdl_v_; // the directional vector of the line
	Eigen::Vector3d  eigen_v_;

	KDL::Vector      kdl_p_; // the offset
	Eigen::Vector3d  eigen_p_;

	double 		     l_; // the length of the line segment, 
	                     // negative if the line is infinite



private:

	// No copying of this class is allowed !
	GeometricLine(const GeometricLine& other) = delete;
	GeometricLine(GeometricLine&& other) = delete;
	GeometricLine& operator=(const GeometricLine& other) = delete;
	GeometricLine& operator=(GeometricLine&& other) noexcept = delete;


};











} // namespace hiqp

#endif // include guard