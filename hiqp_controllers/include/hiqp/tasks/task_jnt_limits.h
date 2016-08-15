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
 * \file   task_jnt_limits.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_JNT_LIMITS_H
#define HIQP_TASK_JNT_LIMITS_H


// HiQP Includes
#include <hiqp/task_function.h>

// STL Includes
#include <string>






namespace hiqp
{







/*!
 * \class TaskJntLimit
 * \brief Represents a task that limits joint velocities
 */	
class TaskJntLimits : public TaskFunction
{

public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	TaskJntLimits() {}




	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~TaskJntLimits() noexcept {}




	/*!
     * \brief <i>Pure virtual</i>. Initializes the task
     *
     * \return 0 upon success
     */
	int init
     (
          const std::vector<std::string>& parameters,
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
		const KDL::Tree& kdl_tree, 
		const KDL::JntArrayVel& kdl_joint_pos_vel
	);




     /*!
     * \brief <i>Pure virtual</i>. Computes all performance measures used when
     *        monitoring this task.
     *
     * \return 0 if the calculation was successful
     */
    int monitor();



private:

	// No copying of this class is allowed !
	TaskJntLimits(const TaskJntLimits& other) = delete;
	TaskJntLimits(TaskJntLimits&& other) = delete;
	TaskJntLimits& operator=(const TaskJntLimits& other) = delete;
	TaskJntLimits& operator=(TaskJntLimits&& other) noexcept = delete;

	//enum LimitationType {kPos = 0, kVel = 1, kAcc = 2};

	unsigned int            num_controls_;


};






} // namespace hiqp

#endif // include guard