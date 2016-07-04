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

    virtual int init(const std::vector<std::string>& parameters) = 0;

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

    virtual int draw() = 0;
    


    






    protected:

    double              e_; // the task function
     
    Eigen::MatrixXd     J_; // the task jacobian

    



    

    







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

    inline void setPriority(unsigned int priority)
    { priority_ = priority; }



    /*!
     * \brief This is called from TaskManager, makes calls to Task::apply()
     *        and TaskBehaviour::apply() to get the new controls.
     *        New tasks and task behaviours are created by implementing the 
     *        apply function of the respective classes.
     */
    int getControls
    (
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel,
        std::vector<double>& controls
    )
    {
        apply(kdl_tree, kdl_joint_pos_vel);
        // The results are stored in protected members e and J

        //std::cout << "e = " << e << "\n";
        //std::cout << "J = " << J << "\n";

        task_behaviour_->apply(e_, J_, controls);

        return 0;
    }




    TaskBehaviour*      task_behaviour_; // pointer to the task behaviour

    unsigned int        priority_;

    bool                visibility_;




    friend              TaskManager;



};











} // namespace hiqp

#endif // include guard