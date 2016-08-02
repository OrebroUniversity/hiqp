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
 * \file   geometric_primitive_map.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_GEOMETRIC_PRIMITIVE_MAP_H
#define HIQP_GEOMETRIC_PRIMITIVE_MAP_H


#include <hiqp/visualizer.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_line.h>
#include <hiqp/geometric_primitives/geometric_plane.h>
#include <hiqp/geometric_primitives/geometric_box.h>
#include <hiqp/geometric_primitives/geometric_cylinder.h>
#include <hiqp/geometric_primitives/geometric_sphere.h>

#include <map>





namespace hiqp
{






/*!
 * \class GeometricPrimitiveMap
 * \brief 
 */  
class GeometricPrimitiveMap
{

public:

	GeometricPrimitiveMap(Visualizer* visualizer)
	: visualizer_(visualizer)
	{}

	~GeometricPrimitiveMap() noexcept {}


	int addGeometricPrimitive
	(
		const std::string& name,
        const std::string& type,
        const std::string& frame_id,
        bool visible,
        const std::vector<double>& color,
        const std::vector<std::string>& parameters
    );

	template<typename PrimitiveType>
	PrimitiveType* getGeometricPrimitive(const std::string& name);

	void redrawAllPrimitives();




private:

	// No copying of this class is allowed !
	GeometricPrimitiveMap(const GeometricPrimitiveMap& other) = delete;
	GeometricPrimitiveMap(GeometricPrimitiveMap&& other) = delete;
	GeometricPrimitiveMap& operator=(const GeometricPrimitiveMap& other) = delete;
	GeometricPrimitiveMap& operator=(GeometricPrimitiveMap&& other) noexcept = delete;





	std::map< std::string, GeometricPoint* > 		point_map_;
	std::map< std::string, GeometricLine* > 	    line_map_;
	std::map< std::string, GeometricPlane* > 		plane_map_;
	std::map< std::string, GeometricBox* > 			box_map_;
	std::map< std::string, GeometricCylinder* > 	cylinder_map_;
	std::map< std::string, GeometricSphere* > 		sphere_map_;




	Visualizer* 								    visualizer_;


};











} // namespace hiqp

#endif // include guard