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
#include <hiqp/hiqp_solver.h>
#include <hiqp/casadi_solver.h>

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
	TaskManager(TaskVisualizer* task_visualizer);

	/*!
     * \brief Destructor
     * Destructs my awesome manager
     */
	~TaskManager() noexcept;

     inline void setNumControls(unsigned int numControls)
     { numControls_ = numControls; }

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
     * \param numControls : the total number of controls used by the controller
     *
     * \return 0 if the task creation was successful,
     *         -1 is the task name was not recognised,
     *         -2 if the task behaviour name was not recognised
     */
     std::size_t addTask(const std::string& task_type,
                         const std::string& behaviour_type,
                         const std::vector<std::string>& behaviour_parameters,
                         const std::string& task_name,
                         unsigned int priority,
                         bool visibility,
                         const std::vector<std::string>& parameters);

     int removeTask(std::size_t task_id);

private:

	// No copying of this class is allowed !
	TaskManager(const TaskManager& other) = delete;
	TaskManager(TaskManager&& other) = delete;
	TaskManager& operator=(const TaskManager& other) = delete;
	TaskManager& operator=(TaskManager&& other) noexcept = delete;

     Task* buildTask(const std::string& task_name);

     TaskBehaviour* buildTaskBehaviour(const std::string& behaviour_name);



     typedef std::map< std::size_t, Task* >            TaskMap;
     typedef std::map< std::size_t, Task* >::iterator  TaskMapIterator;
     typedef std::pair< std::size_t, Task* >           TaskMapElement;


     TaskMap                                      tasks_;
     std::size_t                                  next_task_id_;

     std::map< std::size_t, TaskBehaviour* >      task_behaviours_;
     std::size_t                                  next_task_behaviour_id_;

     TaskVisualizer*                              task_visualizer_;

     HiQPSolver*                                  solver_;

     unsigned int                                 numControls_;

};

} // namespace hiqp

#endif