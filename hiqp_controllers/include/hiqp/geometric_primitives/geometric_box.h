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
		assert(size == 6 || size == 10);

		d1_(0) = std::stod( parameters.at(0) );
		d1_(1) = std::stod( parameters.at(1) );
		d1_(2) = std::stod( parameters.at(2) );

		d2_(0) = std::stod( parameters.at(3) );
		d2_(1) = std::stod( parameters.at(4) );
		d2_(2) = std::stod( parameters.at(5) );

		c_ = 0.5 * (d1_ + d2_);
		dim_ = d2_ - d1_;

		if (size == 10)
		{
			rot_vec_(0) = std::stod( parameters.at(6) );
			rot_vec_(1) = std::stod( parameters.at(7) );
			rot_vec_(2) = std::stod( parameters.at(8) );
			rot_vec_.Normalize();
			a_ = std::stod( parameters.at(9) );
		}
		else
		{
			rot_vec_(0) = 0;
			rot_vec_(1) = 0;
			rot_vec_(2) = 0;
			a_ = 0;
		}
	}



	/*!
     * \brief Destructor
     */
	~GeometricBox() noexcept {}

	inline const KDL::Vector& getOffset1()
	{ return d1_; }

	inline const KDL::Vector& getOffset2()
	{ return d2_; }

	inline const KDL::Vector& getRotationVector()
	{ return rot_vec_; }

	inline double getRotationAngle()
	{ return a_; }

	inline double getCenterX() { return c_(0); }
	inline double getCenterY() { return c_(1); }
	inline double getCenterZ() { return c_(2); }

	inline double getRotationX() { return rot_vec_(0); }
	inline double getRotationY() { return rot_vec_(1); }
	inline double getRotationZ() { return rot_vec_(2); }

	inline double getDimX() { return dim_(0); }
	inline double getDimY() { return dim_(1); }
	inline double getDimZ() { return dim_(2); }



protected:

	KDL::Vector   d1_; // one corner of the box

	KDL::Vector   d2_; // the diagonally opposite corner of the box

	KDL::Vector   c_; // the geometrical cetrum of the box

	KDL::Vector   dim_; // the dimensions fo the box

	KDL::Vector   rot_vec_; // the rotational vector

	double        a_; // the angle of rotation around the rotational vector




private:

	// No copying of this class is allowed !
	GeometricBox(const GeometricBox& other) = delete;
	GeometricBox(GeometricBox&& other) = delete;
	GeometricBox& operator=(const GeometricBox& other) = delete;
	GeometricBox& operator=(GeometricBox&& other) noexcept = delete;

};











} // namespace hiqp

#endif // include guard