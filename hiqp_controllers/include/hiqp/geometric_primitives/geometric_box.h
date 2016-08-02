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
 * \file   geometric_box.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_BOX_H
#define HIQP_GEOMETRIC_BOX_H




#include <hiqp/geometric_primitives/geometric_primitive.h>

#include <kdl/frames.hpp>





namespace hiqp
{




/*!
 * \class GeometricBox
 * \brief 
 * 
 * The angle is the box's clockwise angle of rotation around the normal vector 
 * of the upper side while looking in the same direction as the normal vector.
 * 
 * box:          [x, y, z, w, d, h, nx, ny, nz, a]
 */  
class GeometricBox : public GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricBox
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
		assert(size == 10);

		c_(0) = std::stod( parameters.at(0) );
		c_(1) = std::stod( parameters.at(1) );
		c_(2) = std::stod( parameters.at(2) );

		dim_(0) = std::stod( parameters.at(3) );
		dim_(1) = std::stod( parameters.at(4) );
		dim_(2) = std::stod( parameters.at(5) );

		n_up_(0) = std::stod( parameters.at(6) );
		n_up_(1) = std::stod( parameters.at(7) );
		n_up_(2) = std::stod( parameters.at(8) );
		n_up_.Normalize();
		
		a_ = std::stod( parameters.at(9) );
		

		// Calculate the normal of the left side of the box

		KDL::Vector left = KDL::Vector(0, 0, 1) * n_up_;
		
		KDL::Vector back = n_up_ * left;

		n_left_ = std::cos(a_) * left + std::sin(a_) * back;
	}



	/*!
     * \brief Destructor
     */
	~GeometricBox() noexcept {}

	inline const KDL::Vector& getCentrum() { return c_; }

	inline const KDL::Vector& getDimensions() { return dim_; }

	inline const KDL::Vector& getNormalUp() { return n_up_; }

	inline const KDL::Vector& getNormalLeft() { return n_left_; }

	inline double getRotationAngle() { return a_; }


	inline double getCenterX() { return c_(0); }
	inline double getCenterY() { return c_(1); }
	inline double getCenterZ() { return c_(2); }

	inline double getDimX() { return dim_(0); }
	inline double getDimY() { return dim_(1); }
	inline double getDimZ() { return dim_(2); }

	inline double getNormalUpX() { return n_up_(0); }
	inline double getNormalUpY() { return n_up_(1); }
	inline double getNormalUpZ() { return n_up_(2); }

	inline double getNormalLeftX() { return n_left_(0); }
	inline double getNormalLeftY() { return n_left_(1); }
	inline double getNormalLeftZ() { return n_left_(2); }

	



protected:

	KDL::Vector   c_; // the geometrical cetrum of the box

	KDL::Vector   dim_; // the dimensions of the box


	KDL::Vector   n_up_; // the normal vector of the top plane of the box

	KDL::Vector   n_left_; // the normal vector of the left plane of the box


	double        a_; // the angle of rotation around the normal-up-vector




private:

	// No copying of this class is allowed !
	GeometricBox(const GeometricBox& other) = delete;
	GeometricBox(GeometricBox&& other) = delete;
	GeometricBox& operator=(const GeometricBox& other) = delete;
	GeometricBox& operator=(GeometricBox&& other) noexcept = delete;

};











} // namespace hiqp

#endif // include guard