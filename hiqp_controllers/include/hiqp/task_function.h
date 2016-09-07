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
 * \file   task_function.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_FUNCTION_H
#define HIQP_TASK_FUNCTION_H

// HiQP Includes
#include <hiqp/task_dynamics.h>
#include <hiqp/visualizer.h>
#include <hiqp/geometric_primitive_map.h>

// STL Includes
#include <vector>
#include <iostream>
#include <chrono>

// Eigen Includes
#include <Eigen/Dense>

// Orocos KDL Includes
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>






namespace hiqp
{




    class TaskFactory;
    class TaskManager;


    class TaskFunction
    {
    public:

	/*!
     * \brief Constructor
     * Constructs my awesome task
     */
    TaskFunction() {}



	/*!
     * \brief Destructor
     * Destructs my awesome task
     */
    ~TaskFunction() noexcept {}





    virtual int init
    (
        const std::chrono::steady_clock::time_point& sampling_time,
        const std::vector<std::string>& parameters,
        const KDL::Tree& kdl_tree, 
        unsigned int num_controls
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
        const std::chrono::steady_clock::time_point& sampling_time,
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel
    ) = 0;



    /*!
     * \brief <i>Pure virtual</i>. Computes all performance measures used when
     *        monitoring this task.
     *
     * \return 0 if the calculation was successful
     */
    virtual int monitor() = 0;




    Eigen::VectorXd getInitialState()
    {
        return e_initial_;
    }




    Eigen::MatrixXd getInitialStateJacobian()
    {
        return J_initial_;
    }



    virtual Eigen::VectorXd getFinalState
    (
        const KDL::Tree& kdl_tree
    )
    {
        return Eigen::VectorXd::Zero(e_.rows());
    }



    






    protected:

    Eigen::VectorXd                 e_; // the task function

    Eigen::VectorXd                 e_initial_;
     
    Eigen::MatrixXd                 J_; // the task jacobian

    Eigen::MatrixXd                 J_initial_;

    Eigen::VectorXd                 e_dot_star_; // the task dynamics

    std::vector<int>                task_types_; // -1 leq, 0 eq, 1 geq

    std::vector<double>             performance_measures_;

    std::vector<double>             measures_e_;

    std::vector<double>             measures_e_dot_star_;

    GeometricPrimitiveMap*          geometric_primitive_map_;

    





    

    

    Visualizer* getVisualizer()
    { return visualizer_; }

    GeometricPrimitiveMap* getGeometricPrimitiveMap()
    { return geometric_primitive_map_; }

    // inline std::size_t getId()
    // { return id_; }

    inline const std::string& getTaskName()
    { return task_name_; }

    inline unsigned int getPriority()
    { return priority_; }
    
    inline bool getVisibility()
    { return visibility_; }

    



    

    







    private:

	// No copying of this class is allowed !
    TaskFunction(const TaskFunction& other) = delete;
    TaskFunction(TaskFunction&& other) = delete;
    TaskFunction& operator=(const TaskFunction& other) = delete;
    TaskFunction& operator=(TaskFunction&& other) noexcept = delete;






    inline void setTaskDynamics(TaskDynamics* task_dynamics)
    { task_dynamics_ = task_dynamics; }

    inline void setVisualizer(Visualizer* visualizer)
    { visualizer_ = visualizer; }

    inline void setGeometricPrimitiveMap(GeometricPrimitiveMap* map)
    { geometric_primitive_map_ = map; }

    //inline void setId(std::size_t id)
    //{ id_ = id; }

    inline void setDynamicsId(std::size_t id)
    { dynamics_id_ = id; }

    inline void setTaskName(const std::string& name)
    { task_name_ = name; }

    inline void setPriority(unsigned int priority)
    { priority_ = priority; }

    inline void setVisibility(bool visibility) 
    { visibility_ = visibility; }




/*
    inline const std::vector<double>& getMeasuresE()
    { return measures_e_; }

    inline const std::vector<double>& getMeasuresEDotStar()
    { return measures_e__dot_star_; }
*/


    
    void monitorFunctionAndDynamics()
    {
        measures_e_.clear();
        for (int i=0; i<e_.rows(); ++i)
            measures_e_.push_back( e_(i) );

        measures_e_dot_star_.clear();
        for (int i=0; i<e_dot_star_.rows(); ++i)
            measures_e_dot_star_.push_back( e_dot_star_(i) );
    }



    /*!
     * \brief This is called from TaskFactory, makes calls to Task::apply()
     *        to get the inital task function and jacobain states.
     *        Do not change!
     */
    int computeInitialState
    (
        const std::chrono::steady_clock::time_point& sampling_time,
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel
    )
    {
        apply(sampling_time, kdl_tree, kdl_joint_pos_vel);
        e_initial_ = e_;
        J_initial_ = J_;
        return 0;
    }



    /*!
     * \brief This is called from TaskManager, makes calls to Task::apply()
     *        and TaskBehaviour::apply() to get the new controls.
     *        Do not change!
     */
    int computeTaskMetrics
    (
        const std::chrono::steady_clock::time_point& sampling_time,
        const KDL::Tree& kdl_tree, 
        const KDL::JntArrayVel& kdl_joint_pos_vel
    )
    {
        apply(sampling_time, kdl_tree, kdl_joint_pos_vel);

        task_dynamics_->apply(sampling_time, e_, J_, e_dot_star_);

        return 0;
    }







    TaskDynamics*           task_dynamics_; // pointer to the task behaviour

    Visualizer*             visualizer_;

    //std::size_t             id_; // unique identifier of the task

    // unique identifier of the task dynamics associated with this task
    std::size_t             dynamics_id_; 

    std::string             task_name_; // unique task name identifier

    unsigned int            priority_;

    bool                    visibility_;




    

    friend                  TaskFactory;

    friend                  TaskManager;





};











} // namespace hiqp

#endif // include guard