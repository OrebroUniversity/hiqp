#ifndef HIQP_TASK_H
#define HIQP_TASK_H

/*!
 * \file task.h
 * \brief The abstract interface for tasks
 * \author Marcus A Johansson
 * \version 0.1
 * \date 2016-06-29
 */




// Orocos KDL Includes
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>






namespace hiqp
{






class Task
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	Task() {}

	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~Task() noexcept {}

	/*!
     * \brief <i>Pure virtual</i>. Calculates the task function and task 
     *        jacobian values.
     *
     * \param kdl_tree          : reference to the kinematic dynamic tree of  
     *                            the robot
     * \param kdl_joint_pos_vel : reference to the current joint positions and
     *                            velocities
     * \param task_fun_val      : reference to where the task function value 
     *                            should be stored
     * \param task_jac_val      : reference to where the task jacobian value
     *                            should be stored
     *
     * \return true if the calculation was successful
     */
	virtual bool apply
	(
		const KDL::Tree& kdl_tree, 
		const KDL::JntArrayVel& kdl_joint_pos_vel,
		double& task_fun_val,
		double& task_jac_val
	) = 0;






private:

	// No copying of this class is allowed !
	Task(const Task& other) = delete;
	Task(Task&& other) = delete;
	Task& operator=(const Task& other) = delete;
	Task& operator=(Task&& other) noexcept = delete;





};











} // namespace hiqp

#endif // include guard