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

#ifndef HIQP_JOINT_VELOCITY_CONTROLLER_H
#define HIQP_JOINT_VELOCITY_CONTROLLER_H

#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <hiqp/task_manager.h>
#include <hiqp/hiqp_time_point.h>

#include <hiqp_ros/base_controller.h>
#include <hiqp_ros/ros_visualizer.h>
#include <hiqp_ros/ros_topic_subscriber.h>

#include <hiqp_msgs/SetTask.h>
#include <hiqp_msgs/UpdateTask.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/ListAllTasks.h>
#include <hiqp_msgs/AddGeometricPrimitive.h>
#include <hiqp_msgs/RemoveGeometricPrimitive.h>
#include <hiqp_msgs/RemoveAllGeometricPrimitives.h>

#include <fstream>

namespace hiqp_ros
{

  /*! \brief The standard joint-velocity-controller in ROS */
  typedef 
  controller_interface::Controller<hardware_interface::VelocityJointInterface>
  JointVelocityController;

  /*! \brief The standard joint-velocity hardware interface in ROS */
  typedef 
  hardware_interface::VelocityJointInterface 
  JointVelocityInterface;

  /*! \brief A joint velocity controller that provides full access to the HiQP control framework
   *  \author Marcus A Johansson */  
  class HiQPJointVelocityController : public JointVelocityController {
  public:
    HiQPJointVelocityController();
    ~HiQPJointVelocityController() noexcept;
    
    bool init(JointVelocityInterface *hw, ros::NodeHandle &controller_nh);
    void starting(const ros::Time& time);
    void update(const ros::Time& time, const ros::Duration& period);
    void stopping(const ros::Time& time);

  private:
    HiQPJointVelocityController(const HiQPJointVelocityController& other) = delete;
    HiQPJointVelocityController(HiQPJointVelocityController&& other) = delete;
    HiQPJointVelocityController& operator=(const HiQPJointVelocityController& other) = delete;
    HiQPJointVelocityController& operator=(HiQPJointVelocityController&& other) noexcept = delete;

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

    bool setTask(hiqp_msgs::SetTask::Request& req, hiqp_msgs::SetTask::Response& res);
    bool removeTask(hiqp_msgs::RemoveTask::Request& req, hiqp_msgs::RemoveTask::Response& res);
    bool removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, hiqp_msgs::RemoveAllTasks::Response& res);
    bool listAllTasks(hiqp_msgs::ListAllTasks::Request& req, hiqp_msgs::ListAllTasks::Response& res);
    bool addGeometricPrimitive(hiqp_msgs::AddGeometricPrimitive::Request& req, hiqp_msgs::AddGeometricPrimitive::Response& res);
    bool removeGeometricPrimitive(hiqp_msgs::RemoveGeometricPrimitive::Request& req, hiqp_msgs::RemoveGeometricPrimitive::Response& res);
    bool removeAllGeometricPrimitives(hiqp_msgs::RemoveAllGeometricPrimitives::Request& req, hiqp_msgs::RemoveAllGeometricPrimitives::Response& res);

    typedef std::map<unsigned int, hardware_interface::JointHandle > JointHandleMap;

    std::ofstream logfile_;

    bool                                              is_active_;
    double                                            fps_; // Hz
    double                                            time_since_last_sampling_; // seconds

    bool                                              monitoring_active_;
    double                                            monitoring_publish_rate_;
    ros::Time                                         last_monitoring_update_;

    ros::NodeHandle                                   controller_nh_;
    ros::Publisher                                    monitoring_pub_;

    ROSTopicSubscriber                                topic_subscriber_;

    ros::ServiceServer                                set_task_service_;
    ros::ServiceServer                                update_task_service_;
    ros::ServiceServer                                remove_task_service_;
    ros::ServiceServer                                remove_all_tasks_service_;
    ros::ServiceServer                                list_all_tasks_service_;
    ros::ServiceServer                                add_geomprim_service_;
    ros::ServiceServer                                remove_geomprim_service_;
    ros::ServiceServer                                remove_all_geomprims_service_;

    hardware_interface::VelocityJointInterface *      hardware_interface_;
    JointHandleMap                                    joint_handles_map_;
    std::mutex                                        handles_mutex_;

    std::shared_ptr<Visualizer>                       visualizer_;
    ROSVisualizer                                     ros_visualizer_;
    hiqp::TaskManager                                 task_manager_;

    hiqp::RobotState                                  robot_state_data_;
    hiqp::RobotStatePtr                               robot_state_ptr_;

    std::vector<double>                               output_controls_;
    unsigned int                                      n_controls_;
  };

} // namespace hiqp

#endif // include guard
