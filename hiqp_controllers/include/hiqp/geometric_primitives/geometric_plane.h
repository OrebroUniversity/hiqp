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
		TaskManager* owner,
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color,
		const std::vector<std::string>& parameters
	)
	: GeometricPrimitive(owner, name, frame_id, visible, color)
	{
		assert(parameters.size() == 6);
		x_ = std::stod( parameters.at(0) );
		y_ = std::stod( parameters.at(1) );
		z_ = std::stod( parameters.at(2) );
		nx_ = std::stod( parameters.at(3) );
		ny_ = std::stod( parameters.at(4) );
		nz_ = std::stod( parameters.at(5) );

		if (visible)
		{
			owner_->getTaskVisualizer()->createPlane(frame_id, 
				nx_, ny_, nz_, 0, r_, g_, b_, a_);
		}
	}



	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~GeometricPlane() noexcept {}




private:

	// No copying of this class is allowed !
	GeometricPlane(const GeometricPlane& other) = delete;
	GeometricPlane(GeometricPlane&& other) = delete;
	GeometricPlane& operator=(const GeometricPlane& other) = delete;
	GeometricPlane& operator=(GeometricPlane&& other) noexcept = delete;

	double x_;
	double y_;
	double z_;
	double nx_;
	double ny_;
	double nz_;






};











} // namespace hiqp

#endif // include guard