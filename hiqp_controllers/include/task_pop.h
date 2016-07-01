#ifndef HIQP_TASK_POP_H
#define HIQP_TASK_POP_H

/*!
 * \file task_pop.h
 * \brief The abstract interface for tasks
 * \author Marcus A Johansson
 * \version 0.1
 * \date 2016-06-29
 */

#include <task.h>

// Orocos KDL Includes
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>

// Eigen Includes
#include <Eigen/Dense>






namespace hiqp
{






class TaskPoP : public Task
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	TaskPoP(double n[3], double d);

	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~TaskPoP() noexcept {}

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
	bool apply
	(
		const KDL::Tree& kdl_tree, 
		const KDL::JntArrayVel& kdl_joint_pos_vel,
		double& task_fun_val,
		Eigen::MatrixXd& task_jac_val
	);






private:

	// No copying of this class is allowed !
	TaskPoP(const TaskPoP& other) = delete;
	TaskPoP(TaskPoP&& other) = delete;
	TaskPoP& operator=(const TaskPoP& other) = delete;
	TaskPoP& operator=(TaskPoP&& other) noexcept = delete;

	Eigen::Vector3d n_;
	double d_;



};











} // namespace hiqp

#endif // include guard