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
 * \file   geometric_primitive.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_PRIMITIVE_H
#define HIQP_GEOMETRIC_PRIMITIVE_H

#include <string>
#include <vector>
#include <cassert>


namespace hiqp
{






/*!
 * \class GeometricPrimitive
 * \brief <i>Pure virtual</i>
 */  
class GeometricPrimitive
{
public:

	/*!
     * \brief Constructor
     */
	GeometricPrimitive
	(
		const std::string& name,
		const std::string& frame_id,
		bool visible,
		const std::vector<double>& color
	)
	: name_(name), frame_id_(frame_id), visible_(visible)
	{
		assert(color.size() == 4);
		r_ = color.at(0);
		g_ = color.at(1);
		b_ = color.at(2);
		a_ = color.at(3);
	}



	/*!
     * \brief Destructor
     */
	virtual ~GeometricPrimitive() noexcept = 0;


	inline void setId(unsigned int id) { id_ = id; }
	inline unsigned int getId() { return id_; }

	inline std::string getName() { return name_; }
	inline std::string getFrameId() { return frame_id_; }
	inline bool isVisible() { return visible_; }

	inline double getRedComponent() { return r_; }
	inline double getGreenComponent() { return g_; }
	inline double getBlueComponent() { return b_; }
	inline double getAlphaComponent() { return a_; }


protected:

	std::string		name_;

	std::string 	frame_id_;

	bool 			visible_;

	unsigned int    id_;

	double 			r_, g_, b_, a_;






private:

	// No copying of this class is allowed !
	GeometricPrimitive(const GeometricPrimitive& other) = delete;
	GeometricPrimitive(GeometricPrimitive&& other) = delete;
	GeometricPrimitive& operator=(const GeometricPrimitive& other) = delete;
	GeometricPrimitive& operator=(GeometricPrimitive&& other) noexcept = delete;

};






} // namespace hiqp

#endif // include guard