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
#include <hiqp/task_behaviour.h>
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

     /*!
     * \brief Adds a task to the task manager and activates it
     *
     * \param task_name : name of the task to be created
     * \param behaviour_name : name of the behaviour that shall be associated 
     *                         with this task
     * \param priority : the priority of this task (1 is highest)
     * \param visible : whether this task should be visible or not
     * \param parameters : the task specific parameters
     *
     * \return 0 if the task creation was successful,
     *         -1 is the task name was not recognised,
     *         -2 if the task behaviour name was not recognised
     */
     std::size_t addTask(const std::string& task_name,
                         const std::string& behaviour_name,
                         const std::vector<std::string>& behaviour_parameters,
                         unsigned int priority,
                         bool visibility,
                         const std::vector<std::string>& parameters);

private:

	// No copying of this class is allowed !
	TaskManager(const TaskManager& other) = delete;
	TaskManager(TaskManager&& other) = delete;
	TaskManager& operator=(const TaskManager& other) = delete;
	TaskManager& operator=(TaskManager&& other) noexcept = delete;


     Task* buildTask(const std::string& task_name);

     TaskBehaviour* buildTaskBehaviour(const std::string& behaviour_name);





     std::map< std::size_t, Task* >               tasks_;
     std::size_t                                  next_task_id_;

     std::map< std::size_t, TaskBehaviour* >      task_behaviours_;
     std::size_t                                  next_task_behaviour_id_;

};

} // namespace hiqp

#endif