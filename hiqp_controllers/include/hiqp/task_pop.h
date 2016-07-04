/*!
 * \file   task_pop.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_POP_H
#define HIQP_TASK_POP_H


// HiQP Includes
#include <hiqp/task.h>

// STL Includes
#include <string>






namespace hiqp
{





class TaskPoP : public Task
{
public:




	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	TaskPoP(TaskBehaviour* behaviour,
		    std::string link_name,
		    double nx, double ny, double nz,
		    double d);




	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~TaskPoP() noexcept {}




	/*!
     * \brief <i>Pure virtual</i>. Initializes the task
     *
     * \return 0 upon success
     */
	int init();




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
     * \brief <i>Pure virtual</i>. Draws the task
     *
     * \return 0 upon success
     */
	int draw();





private:

	// No copying of this class is allowed !
	TaskPoP(const TaskPoP& other) = delete;
	TaskPoP(TaskPoP&& other) = delete;
	TaskPoP& operator=(const TaskPoP& other) = delete;
	TaskPoP& operator=(TaskPoP&& other) noexcept = delete;

	std::string			link_name_; // the name of the end-effector link
	Eigen::Vector3d 	n_; // the normal vector of the plane
	double 				d_; // the distance to the plane from the origin



};











} // namespace hiqp

#endif // include guard