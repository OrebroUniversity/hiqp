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
#include <hiqp/hiqp_utils.h>

// STL Includes
#include <iostream>





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
    if (visual_id_map_.find(name) != visual_id_map_.end())
    {
        printHiqpWarning("A primitive with name '" + name 
           + "' already exists. No new primitive was added!");
        return -1;
    }

    std::size_t id = 0;
    bool success = false;

    if (type.compare("point") == 0)
    {
        GeometricPoint* point = new GeometricPoint(
            name, frame_id, visible, color);

        if (point->init(parameters) == 0)
        {
            point_map_.insert( 
                std::pair< std::string, GeometricPoint* >( name, point ) );

            id = visualizer_->add(point);

            success = true;
        }
    }
    else if (type.compare("line") == 0)
    {
        GeometricLine* line = new GeometricLine(
            name, frame_id, visible, color);

        if (line->init(parameters) == 0)
        {
            line_map_.insert( 
                std::pair< std::string, GeometricLine* >( name, line ) );

            id = visualizer_->add(line);

            success = true;
        }
        
    }
    else if (type.compare("plane") == 0)
    {
        GeometricPlane* plane = new GeometricPlane(
            name, frame_id, visible, color);

        if (plane->init(parameters) == 0)
        {
            plane_map_.insert( 
                std::pair< std::string, GeometricPlane* > ( name, plane ) );

            id = visualizer_->add(plane);

            success = true;
        }

    }
    else if (type.compare("box") == 0)
    {
        GeometricBox* box = new GeometricBox(
            name, frame_id, visible, color);

        if (box->init(parameters) == 0)
        {
            box_map_.insert( 
                std::pair< std::string, GeometricBox* > ( name, box ) );

            id = visualizer_->add(box);

            success = true;
        }
    }
    else if (type.compare("cylinder") == 0)
    {
        GeometricCylinder* cylinder = new GeometricCylinder(
            name, frame_id, visible, color);

        if (cylinder->init(parameters) == 0)
        {
            cylinder_map_.insert( 
                std::pair< std::string, GeometricCylinder* > ( name, cylinder ) );

            id = visualizer_->add(cylinder);

            success = true;
        }
    }
    else if (type.compare("sphere") == 0)
    {
        GeometricSphere* sphere = new GeometricSphere(
            name, frame_id, visible, color);

        if (sphere->init(parameters) == 0)
        {
            sphere_map_.insert( 
                std::pair< std::string, GeometricSphere* > ( name, sphere ) );

            id = visualizer_->add(sphere);

            success = true;
        }
    }
    else
    {
        printHiqpWarning("Couldn't parse geometric type '" + type + "'. No new primitive was added!");
        return -2;
    }

    if (!success) return -3;

    visual_id_map_.insert( std::pair< std::string, std::size_t >( name, id ) );
    return 0;
}




/*! \todo primitives should have dependencies on tasks and not be removable if dependencies exists!
 */
int GeometricPrimitiveMap::removeGeometricPrimitive
( 
    std::string name
)
{
    std::size_t id = 0;
    std::map<std::string, std::size_t>::iterator it = visual_id_map_.find(name);
    if (it == visual_id_map_.end())
    {
        printHiqpWarning("While trying to remove primitive with name '" + name 
            + "', could not find that primitive. No action taken!");
        return -1;
    }

    id = it->second;
    visualizer_->remove(id);

    visual_id_map_.erase(name);

    point_map_.erase(name);
    line_map_.erase(name);
    plane_map_.erase(name);
    box_map_.erase(name);
    cylinder_map_.erase(name);
    sphere_map_.erase(name);
    
    return 0;
}



/*! \todo primitives with dependencies to tasks shall not be deleted!
 */
int GeometricPrimitiveMap::clear
()
{
    std::vector<int> ids;

    std::map<std::string, std::size_t>::iterator it = visual_id_map_.begin();
    while (it != visual_id_map_.end())
    {
        ids.push_back(it->second); 
        ++it;
    }

    visualizer_->removeMany(ids);

    visual_id_map_.clear();

    point_map_.clear();
    line_map_.clear();
    plane_map_.clear();
    box_map_.clear();
    cylinder_map_.clear();
    sphere_map_.clear();

    return 0;
}





void GeometricPrimitiveMap::redrawAllPrimitives()
{
    {
        std::map< std::string, GeometricPoint* >::iterator it;
        it = point_map_.begin();
        while (it != point_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricLine* >::iterator it;
        it = line_map_.begin();
        while (it != line_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricPlane* >::iterator it;
        it = plane_map_.begin();
        while (it != plane_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricBox* >::iterator it;
        it = box_map_.begin();
        while (it != box_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricCylinder* >::iterator it;
        it = cylinder_map_.begin();
        while (it != cylinder_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
            ++it;
        }
    }



    {
        std::map< std::string, GeometricSphere* >::iterator it;
        it = sphere_map_.begin();
        while (it != sphere_map_.end())
        {
            std::size_t id = visual_id_map_.find(it->first)->second;
            visualizer_->update(id, it->second);
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


