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
#include <hiqp/hiqp_utils.h>

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
		const std::vector<double>& color
	)
	: GeometricPrimitive(name, frame_id, visible, color)
	{}

	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~GeometricPlane() noexcept {}


	int init(const std::vector<std::string>& parameters)
	{
		assert(parameters.size() == 4);

		kdl_n_(0) = std::stod( parameters.at(0) );
		kdl_n_(1) = std::stod( parameters.at(1) );
		kdl_n_(2) = std::stod( parameters.at(2) );

		kdl_n_.Normalize();

		eigen_n_ << kdl_n_(0), kdl_n_(1), kdl_n_(2);

		d_ = std::stod( parameters.at(3) );
	}



	inline const KDL::Vector&      getNormalKDL()    { return kdl_n_; }

	inline const Eigen::Vector3d&  getNormalEigen()  { return eigen_n_; }

	inline double getOffset() { return d_; }

	inline double getNormalX() { return kdl_n_(0); }
	inline double getNormalY() { return kdl_n_(1); }
	inline double getNormalZ() { return kdl_n_(2); }


protected:

	KDL::Vector      kdl_n_; // the normal vector or the plane
	Eigen::Vector3d  eigen_n_;
	
	double 		  d_; // the offset in the normal direction





private:

	// No copying of this class is allowed !
	GeometricPlane(const GeometricPlane& other) = delete;
	GeometricPlane(GeometricPlane&& other) = delete;
	GeometricPlane& operator=(const GeometricPlane& other) = delete;
	GeometricPlane& operator=(GeometricPlane&& other) noexcept = delete;


};











} // namespace hiqp

#endif // include guard