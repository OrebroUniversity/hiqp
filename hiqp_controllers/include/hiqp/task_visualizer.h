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


#include <cstddef> // std::size_t
#include <map>




namespace hiqp
{




class TaskVisualizer
{
public:

	TaskVisualizer() {}
	~TaskVisualizer() noexcept {}

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




	class TaskVisualPrimitive
	{
	public:
		TaskVisualPrimitive(double r, double g, double b, double a);

		inline void setEsthetics(double r, double g, double b, double a)
		{ r_ = r; g_ = g; b_ = b; a_ = a; }

		double r_; double g_; double b_; double a_;
	};

	class TaskVisualPlane : public TaskVisualPrimitive
	{
	public:
		TaskVisualPlane(double nx, double ny, double nz, double d,
				        double r, double g, double b, double a);

		inline void setGeometry(double nx, double ny, double nz, double d)
		{ nx_ = nx; ny_ = ny; nz_ = nz; d_ = d; }

		double nx_; double ny_; double nz_; double d_;
	};




	typedef std::pair<std::size_t, TaskVisualPrimitive*>           MapElement;
	typedef std::map<std::size_t, TaskVisualPrimitive*>            MapType;
	typedef std::map<std::size_t, TaskVisualPrimitive*>::iterator  MapIterator;




	MapType		primitives_map_;





};





} // namespace hiqp






#endif // include guard