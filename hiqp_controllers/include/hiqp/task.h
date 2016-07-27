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
 * \file   task.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_H
#define HIQP_TASK_H

// HiQP Includes
#include <hiqp/task_behaviour.h>
#include <hiqp/task_visualizer.h>

// STL Includes
#include <vector>
#include <iostream>

// Eigen Includes
#include <Eigen/Dense>

// Orocos KDL Includes
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>






namespace hiqp
{




    class TaskManager; // see task_manager.h for the definition

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

    virtual int init
    (
        const std::vector<std::string>& parameters,
        unsigned int numControls
    ) = 0;

	/*!
     * \brief <i>Pure virtual</i>. Calculates the task function and task 
     *        jacobian values.
     *
     * \param kdl_tree          : reference to the kinematic dynamic tree of  
     *                            the robot
     * \param kdl_joint_pos_vel : reference to the current joint positions and
     *                            velocities
     * \param task_fun_val      : reference to where the output controls are to
     *                            be stored
     *
     * \return 0 if the calculation was successful
     */
    virtual int apply
    (
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel
    ) = 0;



    






    protected:

    Eigen::VectorXd                 e_; // the task function
     
    Eigen::MatrixXd                 J_; // the task jacobian

    Eigen::VectorXd                 e_dot_star_; // the task dynamics

    std::vector<int>                task_types_; // -1 leq, 0 eq, 1 geq

    TaskVisualizer* getTaskVisualizer()
    { return task_visualizer_; }

    



    

    







    private:

	// No copying of this class is allowed !
    Task(const Task& other) = delete;
    Task(Task&& other) = delete;
    Task& operator=(const Task& other) = delete;
    Task& operator=(Task&& other) noexcept = delete;


    inline void setVisibility(bool visibility) 
    { visibility_ = visibility; }

    inline void setTaskBehaviour(TaskBehaviour* task_behaviour)
    { task_behaviour_ = task_behaviour; }

    inline void setTaskVisualizer(TaskVisualizer* task_visualizer)
    { task_visualizer_ = task_visualizer; }

    inline void setPriority(unsigned int priority)
    { priority_ = priority; }

    inline void setId(std::size_t id)
    { id_ = id; }

    inline void setTaskName(const std::string& name)
    { task_name_ = name; }

    inline unsigned int getPriority()
    { return priority_; }

    inline std::size_t getId()
    { return id_; }



    /*!
     * \brief This is called from TaskManager, makes calls to Task::apply()
     *        and TaskBehaviour::apply() to get the new controls.
     *        New tasks and task behaviours are created by implementing the 
     *        apply function of the respective classes.
     */
    int computeTaskMetrics
    (
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel
    )
    {
        apply(kdl_tree, kdl_joint_pos_vel);
        // The results are stored in protected members e_ and J_
        
        task_behaviour_->apply(e_, e_dot_star_);

        return 0;
    }




    TaskBehaviour*      task_behaviour_; // pointer to the task behaviour

    TaskVisualizer*     task_visualizer_;

    std::size_t         id_; // unique identifier of the task

    std::string         task_name_; // non-unique task name

    unsigned int        priority_;

    bool                visibility_;




    friend              TaskManager;



};











} // namespace hiqp

#endif // include guard