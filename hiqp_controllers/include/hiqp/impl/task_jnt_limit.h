/*!
 * \file   task_jnt_limit.h
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







/*!
 * \class TaskJntLimit
 * \brief Represents a task that limits joint positions/velocities/accelerations
 *
 *  The type of limitation (pos/vel/acc) is given as a parameter to the task
 *  along with all lower and upper bounds for each joint.
 */	
class TaskJntLimit : public Task
{

public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
	TaskJntLimit();




	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
	~TaskJntLimit() noexcept {}




	/*!
     * \brief <i>Pure virtual</i>. Initializes the task
     *
     * \return 0 upon success
     */
	int init
     (
          const std::vector<std::string>& parameters,
          unsigned int numControls
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



private:

	// No copying of this class is allowed !
	TaskJntLimit(const TaskJntLimit& other) = delete;
	TaskJntLimit(TaskJntLimit&& other) = delete;
	TaskJntLimit& operator=(const TaskJntLimit& other) = delete;
	TaskJntLimit& operator=(TaskJntLimit&& other) noexcept = delete;

	enum LimitationType {kPos = 0, kVel = 1, kAcc = 2};

	


};






} // namespace hiqp

#endif // include guard