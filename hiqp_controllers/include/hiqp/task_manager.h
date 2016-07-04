/*!
 * \file   task_manager.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */




#ifndef HIQP_TASK_MANAGER_H
#define HIQP_TASK_MANAGER_H

// HiQP Includes
#include <hiqp/task.h>
#include <hiqp/task_visualizer.h>

// STL Includes
#include <vector>

// Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>









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
                               const KDL::JntArrayVel& kdl_joint_pos_vel,
		                     std::vector<double> &controls);

private:

	// No copying of this class is allowed !
	TaskManager(const TaskManager& other) = delete;
	TaskManager(TaskManager&& other) = delete;
	TaskManager& operator=(const TaskManager& other) = delete;
	TaskManager& operator=(TaskManager&& other) noexcept = delete;

     std::vector< Task* > tasks_;	

};

} // namespace hiqp

#endif