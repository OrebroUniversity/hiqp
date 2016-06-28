#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

/*!
 * \file task_manager.h
 * \brief A manager class that provides an interface to all tasks
 * \author Marcus A Johansson
 * \version 0.1
 * \date 2016-06-28
 */

// Boost Includes
//#include <boost/shared_ptr.hpp>

// STL Includes
#include <vector>

// Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>









namespace hiqp {

/*!
 * \class TaskManager
 * \brief Should be created only once!
 *
 *  It's awesome!
 */	
class TaskManager
{
public:

	/*!
     * \brief Constructor
     * Constructs my awesome controller
     */
	TaskManager();

	/*!
     * \brief Destructor
     * Destructs my awesome manager
     */
	~TaskManager() noexcept;

	/*!
     * \brief Called every time the controller is initialized by the 
     *        ros::controller_manager
     *
     * Does some cool stuff!
     *
     * \param kdl_tree : the kinematic dynamic tree of the robot
     * \param n_controls : the number of controls
     * \param controls : reference to the controls data
     * \return true if the initialization was successful
     */
	bool getKinematicControls(const KDL::Tree& kdl_tree,
                               const KDL::JntArray& kdl_joint_pos,
		                     unsigned int n_controls,
		                     std::vector<double> &controls);

private:

	// No copying of this class is allowed !
	TaskManager(const TaskManager& other) = delete;
	TaskManager(TaskManager&& other) = delete;
	TaskManager& operator=(const TaskManager& other) = delete;
	TaskManager& operator=(TaskManager&& other) noexcept = delete;

	

};

} // namespace hiqp

#endif