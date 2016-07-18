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
     * \param d : distance from the plane to the origin
     * \param red : the red color component (0-255)
     * \param blue : the blue color component (0-255)
     * \param green : the green color component (0-255)
     * \param alpha : the alpha component (0-255)
     *  
     * \return the unique identifier of the created task
     */
	std::size_t createPlane
	(
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
     * \return 0 on success, -1 if the identifier was invalid
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
     * \param red : the new red color component (0-255)
     * \param blue : the new blue color component (0-255)
     * \param green : the new green color component (0-255)
     * \param alpha : the new alpha component (0-255)
     *
     * \return 0 on success, -1 if the identifier was not found, -2 if the
     *         identifier was not associated with a plane primitive.
     */
	int setPlaneEsthetics
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