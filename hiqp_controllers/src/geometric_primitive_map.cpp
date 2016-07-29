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
 * \file   geometric_primitive_map.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <hiqp/geometric_primitives/geometric_primitive_map.h>

#include <hiqp/geometric_primitives/geometric_point.h>
#include <hiqp/geometric_primitives/geometric_plane.h>

// STL Includes
//#include <iostream>
#include <cassert>





namespace hiqp {






 /*
# Available types and parameter list syntaxes

# point:        [x, y, z]

# line_vec:     [x, y, z, nx, ny, nz, l]
# line_pp:      [x1, y1, z1, x2, y2, z2]

# plane:        [x, y, z, nx, ny, nz]

# box:          [x1, y1, z1, x2, y2, z2]
# box_rot:      [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]

# cylinder_vec: [x, y, z, nx, ny, nz, height, radius]
# cylinder_pp:  [x1, y1, z1, x2, y2, z2, radius]

# sphere:       [x, y, z, radius]
    */


int GeometricPrimitiveMap::addGeometricPrimitive
(
	const std::string& name,
    const std::string& type,
    const std::string& frame_id,
    bool visible,
    const std::vector<double>& color,
    const std::vector<std::string>& parameters
)
{

    if (type.compare("point") == 0)
    {
        assert( point_map_.find(name) == point_map_.end() );

        GeometricPoint* point = new GeometricPoint(
            name, frame_id, visible, color, parameters);

        point_map_.insert( std::pair< std::string, GeometricPoint* >
            (
                name,
                point
            )
        );

        point->draw(visualizer_);


        
    }/*
    else if (type.compare("line_segment_pp") == 0)
    {
        primitive = new GeometricLineSegment();


        
    }
    else if (type.compare("line_segment_vec") == 0)
    {
        primitive = new GeometricLineSegment();


        
    }*/
    else if (type.compare("plane") == 0)
    {

        assert( plane_map_.find(name) == plane_map_.end() );

        GeometricPlane* plane = new GeometricPlane(
            name, frame_id, visible, color, parameters);

        plane_map_.insert( std::pair< std::string, GeometricPlane* >
            (
                name,
                plane
            )
        );

        plane->draw(visualizer_);


        
    }/*
    else if (type.compare("box") == 0)
    {
        primitive = new GeometricBox();


        
    }
    else if (type.compare("box_rot") == 0)
    {
        primitive = new GeometricBox();


        
    }
    else if (type.compare("cylinder_vec") == 0)
    {
        primitive = new GeometricCylinder();


        
    }
    else if (type.compare("cylinder_pp") == 0)
    {
        primitive = new GeometricCylinder();



    }
    else if (type.compare("sphere") == 0)
    {
        primitive = new GeometricSphere();



    }*/
    else
    {

    }

    return 0;

}










template<>
GeometricPoint* GeometricPrimitiveMap::getGeometricPrimitive<GeometricPoint>
(
	const std::string& name
)
{
	std::map< std::string, GeometricPoint* >::iterator it = 
        point_map_.find(name);

	if (it  == point_map_.end())   return nullptr;
	else                           return it->second;
}



template<>
GeometricPlane* GeometricPrimitiveMap::getGeometricPrimitive<GeometricPlane>
(
    const std::string& name
)
{
    std::map< std::string, GeometricPlane* >::iterator it = 
        plane_map_.find(name);

    if (it  == plane_map_.end())   return nullptr;
    else                           return it->second;
}








} // namespace hiqp


