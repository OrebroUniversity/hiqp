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


#include <hiqp/geometric_primitive_map.h>

// STL Includes
#include <iostream>
#include <cassert>





namespace hiqp {






/*
# Available types and parameter list syntaxes

# point:        [x, y, z]

# line:         [nx, ny, nz x,   y,  z]

# plane:        [x, y, z, nx, ny, nz]

# box:          [x1, y1, z1, x2, y2, z2]
# box:          [x1, y1, z1, x2, y2, z2, nx, ny, nz, angle]

# cylinder:     [nx, ny, nz,  x,  y,  z, height, radius]
# cylinder:     [x1, y1, z1, x2, y2, z2, radius]

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

        point_map_.insert( 
            std::pair< std::string, GeometricPoint* >( name, point ) );

        unsigned int id = visualizer_->add(point);
        point->setId(id);

    }
    else if (type.compare("line") == 0)
    {
        assert( line_map_.find(name) == line_map_.end() );

        GeometricLine* line = new GeometricLine(
            name, frame_id, visible, color, parameters);

        line_map_.insert( 
            std::pair< std::string, GeometricLine* >( name, line ) );

        unsigned int id = visualizer_->add(line);
        line->setId(id);
        
    }
    else if (type.compare("plane") == 0)
    {
        assert( plane_map_.find(name) == plane_map_.end() );

        GeometricPlane* plane = new GeometricPlane(
            name, frame_id, visible, color, parameters);

        plane_map_.insert( 
            std::pair< std::string, GeometricPlane* > ( name, plane ) );

        unsigned int id = visualizer_->add(plane);
        plane->setId(id);

    }
    else if (type.compare("box") == 0)
    {
        assert( box_map_.find(name) == box_map_.end() );

        GeometricBox* box = new GeometricBox(
            name, frame_id, visible, color, parameters);

        box_map_.insert( 
            std::pair< std::string, GeometricBox* > ( name, box ) );

        unsigned int id = visualizer_->add(box);
        box->setId(id);
    }
    else if (type.compare("cylinder") == 0)
    {
        assert( cylinder_map_.find(name) == cylinder_map_.end() );

        GeometricCylinder* cylinder = new GeometricCylinder(
            name, frame_id, visible, color, parameters);

        cylinder_map_.insert( 
            std::pair< std::string, GeometricCylinder* > ( name, cylinder ) );

        unsigned int id = visualizer_->add(cylinder);
        cylinder->setId(id);
    }
    else if (type.compare("sphere") == 0)
    {
        assert( sphere_map_.find(name) == sphere_map_.end() );

        GeometricSphere* sphere = new GeometricSphere(
            name, frame_id, visible, color, parameters);

        sphere_map_.insert( 
            std::pair< std::string, GeometricSphere* > ( name, sphere ) );

        unsigned int id = visualizer_->add(sphere);
        sphere->setId(id);
    }
    else
    {
        std::cerr << "ERROR: Couldn't parse geometric type '" << type << "'!\n";
    }

    return 0;

}




void GeometricPrimitiveMap::redrawAllPrimitives()
{
    {
        std::map< std::string, GeometricPoint* >::iterator it;
        it = point_map_.begin();
        while (it != point_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricLine* >::iterator it;
        it = line_map_.begin();
        while (it != line_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricPlane* >::iterator it;
        it = plane_map_.begin();
        while (it != plane_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricBox* >::iterator it;
        it = box_map_.begin();
        while (it != box_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricCylinder* >::iterator it;
        it = cylinder_map_.begin();
        while (it != cylinder_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricSphere* >::iterator it;
        it = sphere_map_.begin();
        while (it != sphere_map_.end())
        {
            visualizer_->update(it->second->getId(), it->second);
            ++it;
        }
    }
}










template<>
GeometricPoint* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricPoint>
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
GeometricLine* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricLine>
(
    const std::string& name
)
{
    std::map< std::string, GeometricLine* >::iterator it = 
        line_map_.find(name);

    if (it  == line_map_.end())   return nullptr;
    else                          return it->second;
}





template<>
GeometricPlane* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricPlane>
(
    const std::string& name
)
{
    std::map< std::string, GeometricPlane* >::iterator it = 
        plane_map_.find(name);

    if (it  == plane_map_.end())   return nullptr;
    else                           return it->second;
}





template<>
GeometricBox* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricBox>
(
    const std::string& name
)
{
    std::map< std::string, GeometricBox* >::iterator it = 
        box_map_.find(name);

    if (it  == box_map_.end())     return nullptr;
    else                           return it->second;
}





template<>
GeometricCylinder* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricCylinder>
(
    const std::string& name
)
{
    std::map< std::string, GeometricCylinder* >::iterator it = 
        cylinder_map_.find(name);

    if (it  == cylinder_map_.end())     return nullptr;
    else                                return it->second;
}





template<>
GeometricSphere* 
GeometricPrimitiveMap::getGeometricPrimitive<GeometricSphere>
(
    const std::string& name
)
{
    std::map< std::string, GeometricSphere* >::iterator it = 
        sphere_map_.find(name);

    if (it  == sphere_map_.end())       return nullptr;
    else                                return it->second;
}








} // namespace hiqp


