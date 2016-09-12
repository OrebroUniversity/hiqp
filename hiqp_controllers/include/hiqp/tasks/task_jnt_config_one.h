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
 * \file   task_jnt_config_one.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_JNT_CONFIG_ONE_H
#define HIQP_TASK_JNT_CONFIG_ONE_H


// HiQP Includes
#include <hiqp/task_function.h>

// STL Includes
#include <string>






namespace hiqp
{







/*!
 * \class TaskJntConfigOne
 * \brief Represents a task that sets a specific joint configuration
 *
 *  This task does not leave any redundancy available to other tasks.
 */	
class TaskJntConfigOne : public TaskFunction
{

public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	TaskJntConfigOne() {}




	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~TaskJntConfigOne() noexcept {}




	/*!
     * \brief <i>Pure virtual</i>. Initializes the task
     *
     * \return 0 upon success
     */
	int init
     (
          const std::chrono::steady_clock::time_point& sampling_time,
          const std::vector<std::string>& parameters,
          const KDL::Tree& kdl_tree, 
          unsigned int num_controls
     );




	/*!
     * \brief <i>Pure virtual</i>. Calculates the task function and task 
     *        jacobian values.
     *
     * \param kdl_tree : reference to the kinematic dynamic tree of the robot
     * \param joints_pos_vel : reference to the current joint positions and 
     *                         velocities
     *
     * \return true if the calculation was successful
     */
	int apply
	(
          const std::chrono::steady_clock::time_point& sampling_time,
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
	TaskJntConfigOne(const TaskJntConfigOne& other) = delete;
	TaskJntConfigOne(TaskJntConfigOne&& other) = delete;
	TaskJntConfigOne& operator=(const TaskJntConfigOne& other) = delete;
	TaskJntConfigOne& operator=(TaskJntConfigOne&& other) noexcept = delete;


     std::string              joint_name_;
     unsigned int             joint_q_nr_;
     double                   desired_configuration_;

};






} // namespace hiqp

#endif // include guard