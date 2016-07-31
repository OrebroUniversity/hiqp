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
 * \file   visualizer.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_VISUALIZER_H
#define HIQP_VISUALIZER_H





#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line_segment.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>





namespace hiqp
{






class Visualizer
{
public:

	Visualizer() {}

	~Visualizer() noexcept {}

	virtual int add(GeometricPoint* point) = 0;
	virtual int add(GeometricLineSegment* line) = 0;
	virtual int add(GeometricPlane* plane) = 0;
	virtual int add(GeometricBox* box) = 0;
	virtual int add(GeometricCylinder* cylinder) = 0;
	virtual int add(GeometricSphere* sphere) = 0;

	virtual void update(int id, GeometricPoint* point) = 0;
	virtual void update(int id, GeometricLineSegment* line) = 0;
	virtual void update(int id, GeometricPlane* plane) = 0;
	virtual void update(int id, GeometricBox* box) = 0;
	virtual void update(int id, GeometricCylinder* cylinder) = 0;
	virtual void update(int id, GeometricSphere* sphere) = 0;

	virtual void remove(int id) = 0;


private:

	// No copying of this class is allowed !
	Visualizer(const Visualizer& other) = delete;
	Visualizer(Visualizer&& other) = delete;
	Visualizer& operator=(const Visualizer& other) = delete;
	Visualizer& operator=(Visualizer&& other) noexcept = delete;

};





} // namespace hiqp






#endif // include guard