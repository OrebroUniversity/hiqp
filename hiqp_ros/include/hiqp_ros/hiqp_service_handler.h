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

#include <rclcpp/rclcpp.hpp>

#include <hiqp/PrimitiveInfo.h>
#include <hiqp/TaskInfo.h>
#include <hiqp/task_manager.h>

#include <hiqp_msgs/srv/activate_task.hpp>
#include <hiqp_msgs/srv/deactivate_task.hpp>
#include <hiqp_msgs/srv/demonitor_task.hpp>
#include <hiqp_msgs/srv/get_all_tasks.hpp>
#include <hiqp_msgs/srv/list_all_tasks.hpp>
#include <hiqp_msgs/srv/monitor_task.hpp>
#include <hiqp_msgs/srv/remove_all_tasks.hpp>
#include <hiqp_msgs/srv/remove_tasks.hpp>
#include <hiqp_msgs/srv/set_tasks.hpp>

#include <hiqp_msgs/srv/get_all_primitives.hpp>
#include <hiqp_msgs/srv/list_all_primitives.hpp>
#include <hiqp_msgs/srv/remove_all_primitives.hpp>
#include <hiqp_msgs/srv/remove_primitives.hpp>
#include <hiqp_msgs/srv/set_primitives.hpp>

#include <hiqp_msgs/srv/activate_priority_level.hpp>
#include <hiqp_msgs/srv/deactivate_priority_level.hpp>
#include <hiqp_msgs/srv/demonitor_priority_level.hpp>
#include <hiqp_msgs/srv/monitor_priority_level.hpp>
#include <hiqp_msgs/srv/remove_priority_level.hpp>

#include <hiqp_msgs/srv/is_task_set.hpp>

class HiQPServiceHandler {
 public:
  HiQPServiceHandler() = default;

  ~HiQPServiceHandler() noexcept = default;

  void init(rclcpp::Node::SharedPtr node_handle,
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

  //Tasks
  bool setTasks(const std::shared_ptr<hiqp_msgs::srv::SetTasks::Request> req,
      std::shared_ptr<hiqp_msgs::srv::SetTasks::Response> res);
  bool removeTasks(const std::shared_ptr<hiqp_msgs::srv::RemoveTasks::Request> req,
      std::shared_ptr<hiqp_msgs::srv::RemoveTasks::Response> res);
  bool removeAllTasks(const std::shared_ptr<hiqp_msgs::srv::RemoveAllTasks::Request> req,
      std::shared_ptr<hiqp_msgs::srv::RemoveAllTasks::Response> res);
  bool listAllTasks(const std::shared_ptr<hiqp_msgs::srv::ListAllTasks::Request> req,
      std::shared_ptr<hiqp_msgs::srv::ListAllTasks::Response> res);
  bool getAllTasks(const std::shared_ptr<hiqp_msgs::srv::GetAllTasks::Request> req,
      std::shared_ptr<hiqp_msgs::srv::GetAllTasks::Response> res);
  bool activateTask(const std::shared_ptr<hiqp_msgs::srv::ActivateTask::Request> req,
      std::shared_ptr<hiqp_msgs::srv::ActivateTask::Response> res);
  bool deactivateTask(const std::shared_ptr<hiqp_msgs::srv::DeactivateTask::Request> req,
      std::shared_ptr<hiqp_msgs::srv::DeactivateTask::Response> res);
  bool monitorTask(const std::shared_ptr<hiqp_msgs::srv::MonitorTask::Request> req,
      std::shared_ptr<hiqp_msgs::srv::MonitorTask::Response> res);
  bool demonitorTask(const std::shared_ptr<hiqp_msgs::srv::DemonitorTask::Request> req,
      std::shared_ptr<hiqp_msgs::srv::DemonitorTask::Response> res);

  //Primitives
  bool setPrimitives(const std::shared_ptr<hiqp_msgs::srv::SetPrimitives::Request> req,
      std::shared_ptr<hiqp_msgs::srv::SetPrimitives::Response> res);
  bool removePrimitives(const std::shared_ptr<hiqp_msgs::srv::RemovePrimitives::Request> req,
      std::shared_ptr<hiqp_msgs::srv::RemovePrimitives::Response> res);
  bool removeAllPrimitives(const std::shared_ptr<hiqp_msgs::srv::RemoveAllPrimitives::Request> req,
      std::shared_ptr<hiqp_msgs::srv::RemoveAllPrimitives::Response> res);
  bool listAllPrimitives(const std::shared_ptr<hiqp_msgs::srv::ListAllPrimitives::Request> req,
      std::shared_ptr<hiqp_msgs::srv::ListAllPrimitives::Response> res);
  bool getAllPrimitives(const std::shared_ptr<hiqp_msgs::srv::GetAllPrimitives::Request> req,
      std::shared_ptr<hiqp_msgs::srv::GetAllPrimitives::Response> res);

  //Priority levels
  bool removePriorityLevel(const std::shared_ptr<hiqp_msgs::srv::RemovePriorityLevel::Request> req,
      std::shared_ptr<hiqp_msgs::srv::RemovePriorityLevel::Response> res);
  bool activatePriorityLevel(const std::shared_ptr<hiqp_msgs::srv::ActivatePriorityLevel::Request> req,
      std::shared_ptr<hiqp_msgs::srv::ActivatePriorityLevel::Response> res);
  bool deactivatePriorityLevel(
      const std::shared_ptr<hiqp_msgs::srv::DeactivatePriorityLevel::Request> req,
      std::shared_ptr<hiqp_msgs::srv::DeactivatePriorityLevel::Response> res);
  bool monitorPriorityLevel(const std::shared_ptr<hiqp_msgs::srv::MonitorPriorityLevel::Request> req,
      std::shared_ptr<hiqp_msgs::srv::MonitorPriorityLevel::Response> res);
  bool demonitorPriorityLevel(
      const std::shared_ptr<hiqp_msgs::srv::DemonitorPriorityLevel::Request> req,
      std::shared_ptr<hiqp_msgs::srv::DemonitorPriorityLevel::Response> res);

  bool isTaskSet(const std::shared_ptr<hiqp_msgs::srv::IsTaskSet::Request> req,
      std::shared_ptr<hiqp_msgs::srv::IsTaskSet::Response> res);

  rclcpp::Node::SharedPtr node_handle_;
  std::shared_ptr<hiqp::TaskManager> task_manager_;
  hiqp::RobotStatePtr robot_state_;

  std::shared_ptr<rclcpp::ServiceBase> set_tasks_service_;
  std::shared_ptr<rclcpp::ServiceBase> remove_tasks_service_;
  std::shared_ptr<rclcpp::ServiceBase> remove_all_tasks_service_;
  std::shared_ptr<rclcpp::ServiceBase> list_all_tasks_service_;
  std::shared_ptr<rclcpp::ServiceBase> get_all_tasks_service_;
  std::shared_ptr<rclcpp::ServiceBase> activate_task_service_;
  std::shared_ptr<rclcpp::ServiceBase> deactivate_task_service_;
  std::shared_ptr<rclcpp::ServiceBase> monitor_task_service_;
  std::shared_ptr<rclcpp::ServiceBase> demonitor_task_service_;

  std::shared_ptr<rclcpp::ServiceBase> set_primitives_service_;
  std::shared_ptr<rclcpp::ServiceBase> remove_primitives_service_;
  std::shared_ptr<rclcpp::ServiceBase> remove_all_primitives_service_;
  std::shared_ptr<rclcpp::ServiceBase> list_all_primitives_service_;
  std::shared_ptr<rclcpp::ServiceBase> get_all_primitives_service_;

  std::shared_ptr<rclcpp::ServiceBase> remove_priority_level_service_;
  std::shared_ptr<rclcpp::ServiceBase> activate_priority_level_service_;
  std::shared_ptr<rclcpp::ServiceBase> deactivate_priority_level_service_;
  std::shared_ptr<rclcpp::ServiceBase> monitor_priority_level_service_;
  std::shared_ptr<rclcpp::ServiceBase> demonitor_priority_level_service_;
  
  std::shared_ptr<rclcpp::ServiceBase> is_task_set_service_;
};

#endif
