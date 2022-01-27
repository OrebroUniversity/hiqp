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

#include <hiqp/PrimitiveInfo.h>
#include <hiqp/TaskInfo.h>
#include <hiqp/task_manager.h>

#include <hiqp_msgs/ActivateTask.h>
#include <hiqp_msgs/DeactivateTask.h>
#include <hiqp_msgs/DemonitorTask.h>
#include <hiqp_msgs/GetAllTasks.h>
#include <hiqp_msgs/ListAllTasks.h>
#include <hiqp_msgs/MonitorTask.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/RemoveTasks.h>
#include <hiqp_msgs/SetTasks.h>

#include <hiqp_msgs/GetAllPrimitives.h>
#include <hiqp_msgs/ListAllPrimitives.h>
#include <hiqp_msgs/RemoveAllPrimitives.h>
#include <hiqp_msgs/RemovePrimitives.h>
#include <hiqp_msgs/SetPrimitives.h>

#include <hiqp_msgs/ActivatePriorityLevel.h>
#include <hiqp_msgs/DeactivatePriorityLevel.h>
#include <hiqp_msgs/DemonitorPriorityLevel.h>
#include <hiqp_msgs/MonitorPriorityLevel.h>
#include <hiqp_msgs/RemovePriorityLevel.h>

#include <hiqp_msgs/IsTaskSet.h>

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

  /// \todo Add activate_controller ros service
  /// \todo Add deactivate_controller ros service

  bool setTasks(hiqp_msgs::SetTasks::Request& req,
                hiqp_msgs::SetTasks::Response& res);
  bool removeTasks(hiqp_msgs::RemoveTasks::Request& req,
                   hiqp_msgs::RemoveTasks::Response& res);
  bool removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req,
                      hiqp_msgs::RemoveAllTasks::Response& res);
  bool listAllTasks(hiqp_msgs::ListAllTasks::Request& req,
                    hiqp_msgs::ListAllTasks::Response& res);
  bool getAllTasks(hiqp_msgs::GetAllTasks::Request& req,
                   hiqp_msgs::GetAllTasks::Response& res);
  bool activateTask(hiqp_msgs::ActivateTask::Request& req,
                    hiqp_msgs::ActivateTask::Response& res);
  bool deactivateTask(hiqp_msgs::DeactivateTask::Request& req,
                      hiqp_msgs::DeactivateTask::Response& res);
  bool monitorTask(hiqp_msgs::MonitorTask::Request& req,
                   hiqp_msgs::MonitorTask::Response& res);
  bool demonitorTask(hiqp_msgs::DemonitorTask::Request& req,
                     hiqp_msgs::DemonitorTask::Response& res);

  bool setPrimitives(hiqp_msgs::SetPrimitives::Request& req,
                     hiqp_msgs::SetPrimitives::Response& res);
  bool removePrimitives(hiqp_msgs::RemovePrimitives::Request& req,
                        hiqp_msgs::RemovePrimitives::Response& res);
  bool removeAllPrimitives(hiqp_msgs::RemoveAllPrimitives::Request& req,
                           hiqp_msgs::RemoveAllPrimitives::Response& res);
  bool listAllPrimitives(hiqp_msgs::ListAllPrimitives::Request& req,
                         hiqp_msgs::ListAllPrimitives::Response& res);
  bool getAllPrimitives(hiqp_msgs::GetAllPrimitives::Request& req,
                        hiqp_msgs::GetAllPrimitives::Response& res);

  bool removePriorityLevel(hiqp_msgs::RemovePriorityLevel::Request& req,
                           hiqp_msgs::RemovePriorityLevel::Response& res);
  bool activatePriorityLevel(hiqp_msgs::ActivatePriorityLevel::Request& req,
                             hiqp_msgs::ActivatePriorityLevel::Response& res);
  bool deactivatePriorityLevel(
      hiqp_msgs::DeactivatePriorityLevel::Request& req,
      hiqp_msgs::DeactivatePriorityLevel::Response& res);
  bool monitorPriorityLevel(hiqp_msgs::MonitorPriorityLevel::Request& req,
                            hiqp_msgs::MonitorPriorityLevel::Response& res);
  bool demonitorPriorityLevel(hiqp_msgs::DemonitorPriorityLevel::Request& req,
                              hiqp_msgs::DemonitorPriorityLevel::Response& res);
                              
  bool isTaskSet(hiqp_msgs::IsTaskSet::Request& req,
                 hiqp_msgs::IsTaskSet::Response& res);

  std::shared_ptr<ros::NodeHandle> node_handle_;
  std::shared_ptr<hiqp::TaskManager> task_manager_;
  hiqp::RobotStatePtr robot_state_;

  ros::ServiceServer set_tasks_service_;
  ros::ServiceServer remove_tasks_service_;
  ros::ServiceServer remove_all_tasks_service_;
  ros::ServiceServer list_all_tasks_service_;
  ros::ServiceServer get_all_tasks_service_;
  ros::ServiceServer activate_task_service_;
  ros::ServiceServer deactivate_task_service_;
  ros::ServiceServer monitor_task_service_;
  ros::ServiceServer demonitor_task_service_;

  ros::ServiceServer set_primitives_service_;
  ros::ServiceServer remove_primitives_service_;
  ros::ServiceServer remove_all_primitives_service_;
  ros::ServiceServer list_all_primitives_service_;
  ros::ServiceServer get_all_primitives_service_;

  ros::ServiceServer remove_priority_level_service_;
  ros::ServiceServer activate_priority_level_service_;
  ros::ServiceServer deactivate_priority_level_service_;
  ros::ServiceServer monitor_priority_level_service_;
  ros::ServiceServer demonitor_priority_level_service_;
  
  ros::ServiceServer is_task_set_service_;
};

#endif
