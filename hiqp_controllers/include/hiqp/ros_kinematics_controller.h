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
 * \file   ros_kinematics_controller.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#ifndef HIQP_ROS_KINEMATICS_CONTROLLER_H
#define HIQP_ROS_KINEMATICS_CONTROLLER_H

// STL Includes
#include <string>
#include <vector>
#include <mutex>

// ROS Messages

// ROS Services

// ROS Includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
//#include <hardware_interface/posvel_command_interface.h>

// Orocos KDL
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

// HiQP Includes
#include <hiqp/task_manager.h>
#include <hiqp/ros_visualizer.h>
#include <hiqp/ros_topic_subscriber.h>
#include <hiqp/hiqp_time_point.h>

#include <hiqp_msgs_srvs/AddTask.h>
#include <hiqp_msgs_srvs/UpdateTask.h>
#include <hiqp_msgs_srvs/RemoveTask.h>
#include <hiqp_msgs_srvs/RemoveAllTasks.h>
#include <hiqp_msgs_srvs/AddGeometricPrimitive.h>
#include <hiqp_msgs_srvs/RemoveGeometricPrimitive.h>
#include <hiqp_msgs_srvs/RemoveAllGeometricPrimitives.h>



#include <fstream>

namespace hiqp
{

/*! 
 * \brief The standard joint-velocity-controller in ROS 
 */
typedef 
controller_interface::Controller<hardware_interface::VelocityJointInterface>
JointVelocityController;

/*! 
 * \brief The standard joint-velocity hardware interface in ROS 
 */
typedef 
hardware_interface::VelocityJointInterface 
JointVelocityInterface;

/*!
 * \class ROSKinematicsController
 * \brief A velocity controller compatible with ROS that uses the HiQP control framework
 */  
class ROSKinematicsController : public JointVelocityController
{
public:

  ROSKinematicsController();

  ~ROSKinematicsController() noexcept;
  
  /*!
   * \brief Called every time the controller is initialized by the 
   *        ros::controller_manager
   *
   * Does some cool stuff!
   *
   * \param hw : a pointer to the hardware interface used by this controller
   * \param controller_nh : the node handle of this controller
   * \return true if the initialization was successful
   */
  bool init(JointVelocityInterface *hw, 
         ros::NodeHandle &controller_nh);

  /*!
   * \brief Called every time the controller is started by the 
   *        ros::controller_manager
   *
   * Does some cool stuff!
   *
   * \param time : the current wall-time in ROS
   * \return true if the starting was successful
   */
  void starting(const ros::Time& time);

  /*!
   * \brief Called every time the controller is updated by the 
   *        ros::controller_manager
   *
   * The function:
   * <ol>
   *   <li>locks a mutex and reads position and velocity values from the joint handles,</li>
   *   <li>calls getKinematicControls() on its task manager,</li>
   *   <li>and locks a mutex and writes velocity values to the joint handles.</li>
   * </ol>
   * The joint handles are stored as a map between the joints q-number in the 
   * KDL::Tree and the joint handles themselves.
   *
   * \param time : the current wall-time in ROS
   * \param period : the time between the last update call and this, i.e.
   *                 the sample time.
   * \return true if the update was successful
   */
  void update(const ros::Time& time, const ros::Duration& period);

  /*!
   * \brief Called every time the controller is stopped by the 
   *        ros::controller_manager
   *
   * Does some cool stuff!
   *
   * \param time : the current wall-time in ROS
   * \return true if the stopping was successful
   */
  void stopping(const ros::Time& time);



private:
  // No copying of this class is allowed !
  ROSKinematicsController(const ROSKinematicsController& other) = delete;
  ROSKinematicsController(ROSKinematicsController&& other) = delete;
  ROSKinematicsController& operator=(const ROSKinematicsController& other) = delete;
  ROSKinematicsController& operator=(ROSKinematicsController&& other) noexcept = delete;

  void sampleJointValues();

  void setControls();

  void performMonitoring();

  int loadFps();

  int loadAndSetupTaskMonitoring();

  int loadUrdfAndSetupKdlTree();

  int loadJointsAndSetJointHandlesMap();

  void advertiseAllServices();

  void addAllTopicSubscriptions();

  void loadJointLimitsFromParamServer();

  void loadGeometricPrimitivesFromParamServer();

  void loadTasksFromParamServer();


  // These are callback functions for ROS service calls
  bool addTask
  (
    hiqp_msgs_srvs::AddTask::Request& req, 
    hiqp_msgs_srvs::AddTask::Response& res
  );

  bool updateTask
  (
    hiqp_msgs_srvs::UpdateTask::Request& req, 
    hiqp_msgs_srvs::UpdateTask::Response& res
  );

  bool removeTask
  (
    hiqp_msgs_srvs::RemoveTask::Request& req, 
    hiqp_msgs_srvs::RemoveTask::Response& res
  );

  bool removeAllTasks
  (
    hiqp_msgs_srvs::RemoveAllTasks::Request& req, 
    hiqp_msgs_srvs::RemoveAllTasks::Response& res
  );

  bool addGeometricPrimitive
  (
    hiqp_msgs_srvs::AddGeometricPrimitive::Request& req, 
    hiqp_msgs_srvs::AddGeometricPrimitive::Response& res
  );

  bool removeGeometricPrimitive
  (
    hiqp_msgs_srvs::RemoveGeometricPrimitive::Request& req, 
    hiqp_msgs_srvs::RemoveGeometricPrimitive::Response& res
  );

  bool removeAllGeometricPrimitives
  (
    hiqp_msgs_srvs::RemoveAllGeometricPrimitives::Request& req, 
    hiqp_msgs_srvs::RemoveAllGeometricPrimitives::Response& res
  );


  typedef std::map<unsigned int, hardware_interface::JointHandle >
    JointHandleMap;
  typedef std::pair<unsigned int, hardware_interface::JointHandle >
    JointHandleMapEntry;


  std::ofstream logfile_;


  bool                                              is_active_;
  HiQPTimePoint                                     sampling_time_;
  double                                            fps_; // Hz
  double                                            time_since_last_sampling_; // seconds

  bool                                              monitoring_active_;
  double                                            monitoring_publish_rate_;
  ros::Time                                         last_monitoring_update_;

  ros::NodeHandle                                   controller_nh_;
  ros::Publisher                                    monitoring_pub_;

  ROSTopicSubscriber                                topic_subscriber_;

  ros::ServiceServer                                add_task_service_;
  ros::ServiceServer                                update_task_service_;
  ros::ServiceServer                                remove_task_service_;
  ros::ServiceServer                                remove_all_tasks_service_;
  ros::ServiceServer                                add_geomprim_service_;
  ros::ServiceServer                                remove_geomprim_service_;
  ros::ServiceServer                                remove_all_geomprims_service_;

  hardware_interface::VelocityJointInterface *      hardware_interface_;
  JointHandleMap                                    joint_handles_map_;
  std::mutex                                        handles_mutex_;

  ROSVisualizer                                     ros_visualizer_;
  TaskManager                                       task_manager_;

  KDL::Tree                                         kdl_tree_;
  KDL::JntArrayVel                                  kdl_joint_pos_vel_;

  std::vector<double>                               output_controls_;
  unsigned int                                      n_controls_;

}; // class ROSKinematicsController

} // namespace hiqp

#endif // include guard
