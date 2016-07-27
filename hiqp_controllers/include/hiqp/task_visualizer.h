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
 * \file   task_visualizer.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_VISUALIZER_H
#define HIQP_TASK_VISUALIZER_H


// STL Includes
#include <cstddef> // std::size_t
#include <map>


// ROS Includes
#include <ros/ros.h>






namespace hiqp
{

















class TaskVisualPrimitive;

class TaskVisualizer
{
public:

	TaskVisualizer();

	~TaskVisualizer() noexcept {}





	int init(ros::NodeHandle* controller_nh);

     int redraw();





	/*!
     * \brief Sets the visibility of the object (more efficient than setting 
     *        alpha to zero)
     *
     * \param id : the identifier of the visual primitive
     * \param visibility : the new visibility value
     *  
     * \return 0 on success, -1 if the primitive was not found
     */
	int setVisibility(std::size_t id, bool visibility);





	/*!
     * \brief Creates and registers a plane among the objects to be visualized.
     *
     * \param nx : x-component of the normal vector of the plane
     * \param ny : y-component of the normal vector of the plane
     * \param nz : z-component of the normal vector of the plane
     * \param d : distance from the plane to the origin (offset)
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
	std::size_t createPlane
	(
          const std::string& base_link_name,
		double nx, double ny, double nz, double d, 
		double r, double g, double b, double a
	);

	/*!
     * \brief Sets an already existing planes geometry
     *
     * \param id : the unique identifier of the plane
     * \param nx : the new x-component of the normal vector of the plane
     * \param ny : the new y-component of the normal vector of the plane
     * \param nz : the new z-component of the normal vector of the plane
     * \param d : the new distance from the plane to the origin
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with a plane primitive.
     */
	int setPlaneGeometry
	(
		std::size_t id, 
		double nx, double ny, double nz, double d
	);

	/*!
     * \brief Sets an already existing planes esthetics
     *
     * \param id : the unique identifier of the plane
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
	int setPlaneEsthetics
	(
		std::size_t id, 
		double r, double g, double b, double a
	);







     /*!
     * \brief Creates and registers a sphere among the objects to be visualized.
     *
     * \param base_link_name : the frame with respect to which the sphere is placed
     * \param x : the x position of the geometrical centrum of the sphere
     * \param y : the y position of the geometrical centrum of the sphere
     * \param z : the z position of the geometrical centrum of the sphere
     * \param radius : the radius of the sphere
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
     std::size_t createSphere
     (
          const std::string& base_link_name,
          double x, double y, double z, double radius,
          double r, double g, double b, double a
     );

     /*!
     * \brief Sets an already existing sphere's geometry
     *
     * \param id : the unique identifier of the sphere
     * \param x : the x position of the geometrical centrum of the sphere
     * \param y : the y position of the geometrical centrum of the sphere
     * \param z : the z position of the geometrical centrum of the sphere
     * \param radius : the radius of the sphere
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with a sphere primitive.
     */
     int setSphereGeometry
     (
          std::size_t id, 
          double x, double y, double z, double radius
     );

     /*!
     * \brief Sets an already existing sphere's esthetics
     *
     * \param id : the unique identifier of the sphere
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
     int setSphereEsthetics
     (
          std::size_t id, 
          double r, double g, double b, double a
     );







     /*!
     * \brief Creates and registers a box among the objects to be visualized.
     *
     * \param base_link_name : the frame with respect to which the box is placed
     * \param x : the x position of the geometrical center of the box
     * \param y : the y position of the geometrical center of the box
     * \param z : the z position of the geometrical center of the box
     * \param w : the width of the box
     * \param h : the height of the box
     * \param d : the depth of the box
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
     std::size_t createBox
     (
          const std::string& base_link_name,
          double x, double y, double z, double w, double h, double d,
          double r, double g, double b, double a
     );

     /*!
     * \brief Sets an already existing box's geometry
     *
     * \param id : the unique identifier of the box
     * \param x : the x position of the geometrical center of the box
     * \param y : the y position of the geometrical center of the box
     * \param z : the z position of the geometrical center of the box
     * \param w : the width of the box
     * \param h : the height of the box
     * \param d : the depth of the box
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with a box primitive.
     */
     int setBoxGeometry
     (
          std::size_t id, 
          double x, double y, double z, double w, double h, double d
     );

     /*!
     * \brief Sets an already existing box's esthetics
     *
     * \param id : the unique identifier of the box
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
     int setBoxEsthetics
     (
          std::size_t id, 
          double r, double g, double b, double a
     );







     /*!
     * \brief Creates and registers an arrow among the objects to be visualized.
     *
     * \param base_link_name : the frame with respect to which the box is placed
     * \param x : the x position of the geometrical center of the arrow
     * \param y : the y position of the geometrical center of the arrow
     * \param z : the z position of the geometrical center of the arrow
     * \param nx : the x-component of the direction of the arrow 
     * \param ny : the y-component of the direction of the arrow 
     * \param nz : the z-component of the direction of the arrow 
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
     std::size_t createArrow
     (
          const std::string& base_link_name,
          double x, double y, double z, double nx, double ny, double nz,
          double r, double g, double b, double a
     );

     /*!
     * \brief Sets an already existing arrows's geometry
     *
     * \param id : the unique identifier of the arrow
     * \param x : the x position of the geometrical center of the arrow
     * \param y : the y position of the geometrical center of the arrow
     * \param z : the z position of the geometrical center of the arrow
     * \param nx : the x-component of the direction of the arrow 
     * \param ny : the y-component of the direction of the arrow 
     * \param nz : the z-component of the direction of the arrow 
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with an arrow primitive.
     */
     int setArrowGeometry
     (
          std::size_t id, 
          double x, double y, double z, double nx, double ny, double nz
     );

     /*!
     * \brief Sets an already existing arrows's esthetics
     *
     * \param id : the unique identifier of the arrow
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
     int setArrowEsthetics
     (
          std::size_t id, 
          double r, double g, double b, double a
     );







     /*!
     * \brief Creates and registers a line among the objects to be visualized.
     *
     * \param base_link_name : the frame with respect to which the line is placed
     * \param x : the x position of the geometrical center of the line
     * \param y : the y position of the geometrical center of the line
     * \param z : the z position of the geometrical center of the line
     * \param nx : the x-component of the direction of the line 
     * \param ny : the y-component of the direction of the line 
     * \param nz : the z-component of the direction of the line 
     * \param l : the length of the line
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
     // std::size_t createLine
     // (
     //      const std::string& base_link_name,
     //      double x, double y, double z, double nx, double ny, double nz,
     //      double l,
     //      double r, double g, double b, double a
     // );

     /*!
     * \brief Sets an already existing arrows's geometry
     *
     * \param id : the unique identifier of the line
     * \param x : the x position of the geometrical center of the line
     * \param y : the y position of the geometrical center of the line
     * \param z : the z position of the geometrical center of the line
     * \param nx : the x-component of the direction of the line 
     * \param ny : the y-component of the direction of the line 
     * \param nz : the z-component of the direction of the line 
     * \param l : the length of the line
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with a line primitive.
     */
     // int setLineGeometry
     // (
     //      std::size_t id, 
     //      double x, double y, double z, double nx, double ny, double nz,
     //      double l
     // );

     /*!
     * \brief Sets an already existing arrows's esthetics
     *
     * \param id : the unique identifier of the line
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
     // int setLineEsthetics
     // (
     //      std::size_t id, 
     //      double r, double g, double b, double a
     // );








     /*!
     * \brief Creates and registers a cylinder among the objects to be visualized.
     *
     * \param base_link_name : the frame with respect to which the cylinder is placed
     * \param x : the x position of the geometrical center of the cylinder
     * \param y : the y position of the geometrical center of the cylinder
     * \param z : the z position of the geometrical center of the cylinder
     * \param radius : the radius of the cylinder
     * \param height : the height of the cylinder
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *  
     * \return the unique identifier of the created task
     */
     std::size_t createCylinder
     (
          const std::string& base_link_name,
          double x, double y, double z, double radius, double height,
          double r, double g, double b, double a
     );

     /*!
     * \brief Sets an already existing cylinder's geometry
     *
     * \param id : the unique identifier of the cylinder
     * \param x : the x position of the geometrical center of the cylinder
     * \param y : the y position of the geometrical center of the cylinder
     * \param z : the z position of the geometrical center of the cylinder
     * \param radius : the radius of the cylinder
     * \param height : the height of the cylinder
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with an cylinder primitive.
     */
     int setCylinderGeometry
     (
          std::size_t id, 
          double x, double y, double z, double radius, double height
     );

     /*!
     * \brief Sets an already existing cylinder's esthetics
     *
     * \param id : the unique identifier of the arrow
     * \param r : the red color component (0.0-1.0)
     * \param g : the green color component (0.0-1.0)
     * \param b : the blue color component (0.0-1.0)
     * \param a : the alpha component (0.0-1.0)
     *
     * \return 0 on success, -1 if the identifier was not found.
     */
     int setCylinderEsthetics
     (
          std::size_t id, 
          double r, double g, double b, double a
     );

	









private:

	// No copying of this class is allowed !
	TaskVisualizer(const TaskVisualizer& other) = delete;
	TaskVisualizer(TaskVisualizer&& other) = delete;
	TaskVisualizer& operator=(const TaskVisualizer& other) = delete;
	TaskVisualizer& operator=(TaskVisualizer&& other) noexcept = delete;

	std::size_t insertPrimitive(TaskVisualPrimitive* primitive);

     int setPrimitiveEsthetics(
          std::size_t id, 
          double r, double g, double b, double a
     );

	typedef std::pair<std::size_t, TaskVisualPrimitive*>           MapElement;
	typedef std::map<std::size_t, TaskVisualPrimitive*>            MapType;
	typedef std::map<std::size_t, TaskVisualPrimitive*>::iterator  MapIterator;


	MapType								primitives_map_;

	std::size_t							next_id_;

	ros::NodeHandle*                             controller_nh_;

	ros::Publisher 						marker_pub_;

};





} // namespace hiqp






#endif // include guard