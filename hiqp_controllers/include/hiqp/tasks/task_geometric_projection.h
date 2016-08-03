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
 * \file   task_geometric_projection.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_GEOMETRIC_PROJECTION_H
#define HIQP_TASK_GEOMETRIC_PROJECTION_H


// HiQP Includes
#include <hiqp/task_function.h>

// STL Includes
#include <string>
#include <vector>






namespace hiqp
{




/*!
 * \class TaskGeometricProjection
 * \brief 
 *
 *  It's awesome!
 */  
template<typename PrimitiveA, typename PrimitiveB>
class TaskGeometricProjection : public TaskFunction
{
public:




	/*!
     * \brief Constructor
     */
	TaskGeometricProjection();




	/*!
     * \brief Destructor
     */
	~TaskGeometricProjection() noexcept {}




	/*!
     * \brief Initializes the task
     *
     * \return 0 upon success
     */
	int init
     (
          const std::vector<std::string>& parameters,
          unsigned int num_controls
     );




	/*!
     * \brief Calculates the task function and task jacobian values.
     *
     * \param kdl_tree : reference to the kinematic dynamic tree of the robot
     * \param joints_pos_vel : reference to the current joint positions and 
     *                         velocities
     *
     * \return true if the calculation was successful
     */
	int apply
	(
		const KDL::Tree& kdl_tree, 
		const KDL::JntArrayVel& kdl_joint_pos_vel
	);




     /*!
     * \brief Computes all performance measures used when monitoring this task.
     *
     * \return 0 if the calculation was successful
     */
     int monitor();







private:

	// No copying of this class is allowed !
	TaskGeometricProjection(const TaskGeometricProjection& other) = delete;
	TaskGeometricProjection(TaskGeometricProjection&& other) = delete;
	TaskGeometricProjection& operator=(const TaskGeometricProjection& other) = delete;
	TaskGeometricProjection& operator=(TaskGeometricProjection&& other) noexcept = delete;



     int project(PrimitiveA* first, PrimitiveB* second);






     PrimitiveA*              primitive_a_;
     KDL::Frame               pose_a_;
     KDL::Jacobian            jacobian_a_;

     PrimitiveB*              primitive_b_;
     KDL::Frame               pose_b_;
     KDL::Jacobian            jacobian_b_;



/*
     // the name of the end-effector link
	std::string			point_frame_id_;

     // p_ is a vector in the end-effector link frame from the link's origin
     // to the end-effector point
     KDL::Vector              p_;

     // the name of the plane link
     std::string              plane_frame_id_;

     // the normal vector of the plane in plane frame coordinates
	KDL::Vector 	          n_;

     // the distance to the plane from the origin of the plane frame
	double 				d_;
*/


};











} // namespace hiqp

#include <hiqp/tasks/task_geometric_projection__impl.h>

#endif // include guard