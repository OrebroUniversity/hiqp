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

#ifndef HIQP_SERVICE_HANDLER_H
#define HIQP_SERVICE_HANDLER_H

#include <ros/ros.h>

#include <hiqp/task_manager.h>

#include <hiqp_msgs/SetTask.h>
#include <hiqp_msgs/UpdateTask.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/ListAllTasks.h>
#include <hiqp_msgs/AddPrimitive.h>
#include <hiqp_msgs/RemovePrimitive.h>
#include <hiqp_msgs/RemoveAllPrimitives.h>
#include <hiqp_msgs/ListAllPrimitives.h>

class HiQPServiceHandler {
public:
  HiQPServiceHandler() = default;

  ~HiQPServiceHandler() noexcept = default;

  void init(std::shared_ptr<ros::NodeHandle> node_handle, 
            std::shared_ptr<hiqp::TaskManager> task_manager,
            hiqp::RobotStatePtr robot_state) {
    node_handle_ = node_handle;
    task_manager_ = task_manager;
    robot_state_ = robot_state;
  }

  void advertiseAll();

private:
  HiQPServiceHandler(const HiQPServiceHandler& other) = delete;
  HiQPServiceHandler(HiQPServiceHandler&& other) = delete;
  HiQPServiceHandler& operator=(const HiQPServiceHandler& other) = delete;
  HiQPServiceHandler& operator=(HiQPServiceHandler&& other) noexcept = delete;

  /// \todo Change add_primitive to set_primitive ros service
  /// \todo Add list_all_primitives ros service
  /// \todo Add show_primitive ros service
  /// \todo Add hide_primitive ros service

  /// \todo Add activate_controller ros service
  /// \todo Add deactivate_controller ros service

  /// \todo Add activate_task ros service
  /// \todo Add deactivate_task ros service
  /// \todo Add show_task ros service
  /// \todo Add hide_task ros service
  /// \todo Add monitor_task ros service
  /// \todo Add unmonitor_task ros service

  /// \todo Add activate_stage ros service
  /// \todo Add deactivate_stage ros service
  /// \todo Add monitor_stage ros service
  /// \todo Add unmonitor_stage ros service

  bool setTask(hiqp_msgs::SetTask::Request& req, hiqp_msgs::SetTask::Response& res);
  bool removeTask(hiqp_msgs::RemoveTask::Request& req, hiqp_msgs::RemoveTask::Response& res);
  bool removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, hiqp_msgs::RemoveAllTasks::Response& res);
  bool listAllTasks(hiqp_msgs::ListAllTasks::Request& req, hiqp_msgs::ListAllTasks::Response& res);

  bool addPrimitive(hiqp_msgs::AddPrimitive::Request& req, hiqp_msgs::AddPrimitive::Response& res);
  bool removePrimitive(hiqp_msgs::RemovePrimitive::Request& req, hiqp_msgs::RemovePrimitive::Response& res);
  bool removeAllPrimitives(hiqp_msgs::RemoveAllPrimitives::Request& req, hiqp_msgs::RemoveAllPrimitives::Response& res);
  bool listAllPrimitives(hiqp_msgs::ListAllPrimitives::Request& req, hiqp_msgs::ListAllPrimitives::Response& res);

  std::shared_ptr<ros::NodeHandle>    node_handle_;
  std::shared_ptr<hiqp::TaskManager>  task_manager_;
  hiqp::RobotStatePtr                 robot_state_;

  ros::ServiceServer                  set_task_service_;
  ros::ServiceServer                  update_task_service_;
  ros::ServiceServer                  remove_task_service_;
  ros::ServiceServer                  remove_all_tasks_service_;
  ros::ServiceServer                  list_all_tasks_service_;
  ros::ServiceServer                  add_primitive_service_;
  ros::ServiceServer                  remove_primitive_service_;
  ros::ServiceServer                  remove_all_primitives_service_;
  ros::ServiceServer                  list_all_primitives_service_;
};

#endif