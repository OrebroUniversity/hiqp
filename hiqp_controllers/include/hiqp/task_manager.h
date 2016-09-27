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

/*
 * \file   task_manager.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_TASK_MANAGER_H
#define HIQP_TASK_MANAGER_H

// STL Includes
#include <vector>

// HiQP Includes
#include <hiqp/task_function.h>
#include <hiqp/task_dynamics.h>
#include <hiqp/task_factory.h>
#include <hiqp/visualizer.h>
#include <hiqp/hiqp_solver.h>
#include <hiqp/hiqp_time_point.h>

#include <hiqp/geometric_primitives/geometric_primitive_map.h>

#include <hiqp/solvers/casadi_solver.h>

// Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/jntarrayvel.hpp>





namespace hiqp {

/*!
 * \class TaskMonitoringData
 * \brief An aggregation of const references to facilitate communication of
 *        monitoring data.
 */  
class TaskMonitoringData
{
public:
  TaskMonitoringData
  (
    const std::string& task_name,
    const std::string& measure_tag,
    const std::vector<double>& performance_measures
    )
  : task_name_(task_name), 
    measure_tag_(measure_tag),
    performance_measures_(performance_measures)
  {}

  const std::string&          task_name_;
  std::string                 measure_tag_;
  const std::vector<double>&  performance_measures_;
};





/*!
 * \class TaskManager
 * \brief The central mediator class in teh HiQP framework
 */  
class TaskManager
{
public:

  TaskManager(Visualizer* visualizer);

  ~TaskManager() noexcept;

  void init(unsigned int num_controls);

  /*!
   * \brief Called every time the controller is updated by the 
   *        ros::controller_manager
   *
   * Does some cool stuff!
   *
   * \param kdl_tree : the kinematic dynamic tree of the robot
   * \param n_controls : the number of controls
   * \param controls : reference to the controls data
   * \return true if the initialization was successful
   */
  bool getKinematicControls
  (
    const HiQPTimePoint& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel,
    std::vector<double> &controls
  );

  /*!
   * \brief Retrieves the performance measures for every active task along
   *        with the task's name and unique identifier.
   *
   * NOTE: The vector sent to this function is supposed to be empty
   *
   * \param data : a reference to a vector of monitoring data structures
   */
   void getTaskMonitoringData(std::vector<TaskMonitoringData>& data);

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
   *         -1 is the behaviour_parameters argument was invalid,
   *         -2 if the task behaviour name was not recognised,
   *         -3 if the task name was not recognised
   */
  int addTask
  (
    const std::string& name,
    const std::string& type,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    bool active,
    const std::vector<std::string>& parameters,
    const HiQPTimePoint& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel
  );

  int updateTask
  (
    const std::string& name,
    const std::string& type,
    const std::vector<std::string>& behaviour_parameters,
    unsigned int priority,
    bool visibility,
    bool active,
    const std::vector<std::string>& parameters,
    const HiQPTimePoint& sampling_time,
    const KDL::Tree& kdl_tree,
    const KDL::JntArrayVel& kdl_joint_pos_vel
  );

  int removeTask(std::string task_name);

  int removeAllTasks();

  int listAllTasks();

  inline void activateTask(const std::string& name)
  { tasks_.find(name)->second->setIsActive(true); }

  inline void deactivateTask(const std::string& name)
  { tasks_.find(name)->second->setIsActive(false); }

  int addGeometricPrimitive
  (
    const std::string& name,
    const std::string& type,
    const std::string& frame_id,
    bool visible,
    const std::vector<double>& color,
    const std::vector<double>& parameters
  );

  int removeGeometricPrimitive(std::string name);

  int removeAllGeometricPrimitives();

  inline GeometricPrimitiveMap* getGeometricPrimitiveMap()
  { return geometric_primitive_map_; }


 private:
  // No copying of this class is allowed !
  TaskManager(const TaskManager& other) = delete;
  TaskManager(TaskManager&& other) = delete;
  TaskManager& operator=(const TaskManager& other) = delete;
  TaskManager& operator=(TaskManager&& other) noexcept = delete;


  typedef std::map< std::string, TaskFunction* >            TaskMap;
  typedef std::map< std::string, TaskFunction* >::iterator  TaskMapIterator;
  typedef std::pair< std::string, TaskFunction* >           TaskMapElement;



  TaskMap                                      tasks_;

  std::vector<std::size_t>                     existing_task_ids_;

  std::map< std::size_t, TaskDynamics* >       task_dynamics_;
  std::size_t                                  next_task_dynamics_id_;

  TaskFactory*                                 task_factory_;

  GeometricPrimitiveMap*                       geometric_primitive_map_;

  Visualizer*                                  visualizer_;

  HiQPSolver*                                  solver_;

  unsigned int                                 num_controls_;

}; // class TaskManager

} // namespace hiqp

#endif