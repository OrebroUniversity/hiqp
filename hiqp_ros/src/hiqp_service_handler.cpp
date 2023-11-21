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

#include <hiqp_ros/hiqp_service_handler.h>

using std::placeholders::_1;
using std::placeholders::_2;

void HiQPServiceHandler::advertiseAll() {
  set_tasks_service_ = node_handle_->create_service<hiqp_msgs::srv::SetTasks>(
      "/hiqp_controller/set_tasks", std::bind(&HiQPServiceHandler::setTasks, this, _1, _2));
  remove_tasks_service_ = node_handle_->create_service<hiqp_msgs::srv::RemoveTasks>(
      "/hiqp_controller/remove_tasks",std::bind(&HiQPServiceHandler::removeTasks, this, _1, _2));
  remove_all_tasks_service_ = node_handle_->create_service<hiqp_msgs::srv::RemoveAllTasks>(
      "/hiqp_controller/remove_all_tasks",std::bind(&HiQPServiceHandler::removeAllTasks, this, _1, _2));
  list_all_tasks_service_ = node_handle_->create_service<hiqp_msgs::srv::ListAllTasks>(
      "/hiqp_controller/list_all_tasks", std::bind(&HiQPServiceHandler::listAllTasks, this, _1, _2));
  get_all_tasks_service_ = node_handle_->create_service<hiqp_msgs::srv::GetAllTasks>(
      "/hiqp_controller/get_all_tasks", std::bind(&HiQPServiceHandler::getAllTasks, this, _1, _2));
  activate_task_service_ = node_handle_->create_service<hiqp_msgs::srv::ActivateTask>(
      "/hiqp_controller/activate_task", std::bind(&HiQPServiceHandler::activateTask, this, _1, _2));
  deactivate_task_service_ = node_handle_->create_service<hiqp_msgs::srv::DeactivateTask>(
      "/hiqp_controller/deactivate_task", std::bind(&HiQPServiceHandler::deactivateTask, this, _1, _2));
  monitor_task_service_ = node_handle_->create_service<hiqp_msgs::srv::MonitorTask>(
      "/hiqp_controller/monitor_task", std::bind(&HiQPServiceHandler::monitorTask, this, _1, _2));
  demonitor_task_service_ = node_handle_->create_service<hiqp_msgs::srv::DemonitorTask>(
      "/hiqp_controller/demonitor_task", std::bind(&HiQPServiceHandler::demonitorTask, this, _1, _2));

  set_primitives_service_ = node_handle_->create_service<hiqp_msgs::srv::SetPrimitives>(
      "/hiqp_controller/set_primitives", std::bind(&HiQPServiceHandler::setPrimitives, this, _1, _2));
  remove_primitives_service_ = node_handle_->create_service<hiqp_msgs::srv::RemovePrimitives>(
      "/hiqp_controller/remove_primitives", std::bind(&HiQPServiceHandler::removePrimitives, this, _1, _2));
  remove_all_primitives_service_ = node_handle_->create_service<hiqp_msgs::srv::RemoveAllPrimitives>(
      "/hiqp_controller/remove_all_primitives", std::bind(&HiQPServiceHandler::removeAllPrimitives, this, _1, _2));
  list_all_primitives_service_ = node_handle_->create_service<hiqp_msgs::srv::ListAllPrimitives>(
      "/hiqp_controller/list_all_primitives",std::bind(&HiQPServiceHandler::listAllPrimitives, this, _1, _2));
  get_all_primitives_service_ = node_handle_->create_service<hiqp_msgs::srv::GetAllPrimitives>(
      "/hiqp_controller/get_all_primitives", std::bind(&HiQPServiceHandler::getAllPrimitives, this, _1, _2));

  remove_priority_level_service_ = node_handle_->create_service<hiqp_msgs::srv::RemovePriorityLevel>(
      "/hiqp_controller/remove_priority_level", std::bind(&HiQPServiceHandler::removePriorityLevel, this, _1, _2));
  activate_priority_level_service_ = node_handle_->create_service<hiqp_msgs::srv::ActivatePriorityLevel>
    ( "/hiqp_controller/activate_priority_level", 
      std::bind(&HiQPServiceHandler::activatePriorityLevel, this, _1, _2));
  deactivate_priority_level_service_ = 
    node_handle_->create_service<hiqp_msgs::srv::DeactivatePriorityLevel>(
      "/hiqp_controller/deactivate_priority_level", 
      std::bind(&HiQPServiceHandler::deactivatePriorityLevel, this, _1, _2));
  monitor_priority_level_service_ = 
    node_handle_->create_service<hiqp_msgs::srv::MonitorPriorityLevel>(
      "/hiqp_controller/monitor_priority_level", 
      std::bind(&HiQPServiceHandler::monitorPriorityLevel, this, _1, _2));
  demonitor_priority_level_service_ = 
    node_handle_->create_service<hiqp_msgs::srv::DemonitorPriorityLevel>(
      "/hiqp_controller/demonitor_priority_level", 
      std::bind(&HiQPServiceHandler::demonitorPriorityLevel, this, _1, _2));

  is_task_set_service_ = node_handle_->create_service<hiqp_msgs::srv::IsTaskSet>(
      "/hiqp_controller/is_task_set",
      std::bind(&HiQPServiceHandler::isTaskSet, this, _1, _2));
}

bool HiQPServiceHandler::setTasks(const std::shared_ptr<hiqp_msgs::srv::SetTasks::Request> req,
    std::shared_ptr<hiqp_msgs::srv::SetTasks::Response> res) {
  for (auto task : req->tasks) {
    
    int retval = task_manager_->setTask(
        task.name, task.priority, task.visible, task.active, task.monitored,
        task.def_params, task.dyn_params, robot_state_);
    res->success.push_back(retval < 0 ? false : true);
  }
  return true;
}

bool HiQPServiceHandler::removeTasks(const std::shared_ptr<hiqp_msgs::srv::RemoveTasks::Request> req,
    std::shared_ptr<hiqp_msgs::srv::RemoveTasks::Response> res) {
  for (auto name : req->names) {
    if (task_manager_->removeTask(name) == 0)
      res->success.push_back(true);
    else
      res->success.push_back(false);

    if (res->success.back()) {
      hiqp::printHiqpInfo("Removed task '" + name + "'.");
    } else {
      hiqp::printHiqpInfo("Couldn't remove task '" + name + "'!");
    }
  }
  return true;
}

bool HiQPServiceHandler::removeAllTasks(
    const std::shared_ptr<hiqp_msgs::srv::RemoveAllTasks::Request> req,
    std::shared_ptr<hiqp_msgs::srv::RemoveAllTasks::Response> res){
  task_manager_->removeAllTasks();
  hiqp::printHiqpInfo("Removed all tasks successfully!");
  res->success = true;
  return true;
}

bool HiQPServiceHandler::listAllTasks(
    const std::shared_ptr<hiqp_msgs::srv::ListAllTasks::Request> req,
    std::shared_ptr<hiqp_msgs::srv::ListAllTasks::Response> res){
  task_manager_->listAllTasks();
  res->success = true;
  return true;
}

bool HiQPServiceHandler::getAllTasks(
    const std::shared_ptr<hiqp_msgs::srv::GetAllTasks::Request> req,
    std::shared_ptr<hiqp_msgs::srv::GetAllTasks::Response> res){
  std::vector<hiqp::TaskInfo> all_task_info = task_manager_->getAllTaskInfo();
  for (auto it : all_task_info) {
    hiqp_msgs::msg::Task t;
    t.name = it.name;
    t.priority = it.priority;
    t.active = it.active;
    t.monitored = it.monitored;
    t.def_params = it.def_params;
    t.dyn_params = it.dyn_params;
    res->tasks.push_back(t);
  }
  return true;
}

bool HiQPServiceHandler::activateTask(
    const std::shared_ptr<hiqp_msgs::srv::ActivateTask::Request> req,
    std::shared_ptr<hiqp_msgs::srv::ActivateTask::Response> res){
  task_manager_->activateTask(req->name);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::deactivateTask(
    const std::shared_ptr<hiqp_msgs::srv::DeactivateTask::Request> req,
    std::shared_ptr<hiqp_msgs::srv::DeactivateTask::Response> res){
  task_manager_->deactivateTask(req->name);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::monitorTask(
    const std::shared_ptr<hiqp_msgs::srv::MonitorTask::Request> req,
    std::shared_ptr<hiqp_msgs::srv::MonitorTask::Response> res){
  task_manager_->monitorTask(req->name);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::demonitorTask(
    const std::shared_ptr<hiqp_msgs::srv::DemonitorTask::Request> req,
    std::shared_ptr<hiqp_msgs::srv::DemonitorTask::Response> res){
  task_manager_->demonitorTask(req->name);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::setPrimitives(
    const std::shared_ptr<hiqp_msgs::srv::SetPrimitives::Request> req,
    std::shared_ptr<hiqp_msgs::srv::SetPrimitives::Response> res){

  for (auto primitive : req->primitives) {
    RCLCPP_INFO_STREAM(node_handle_->get_logger(), "Setting primitive "<<primitive.name<<" of type "<<primitive.type); 
    int retval = task_manager_->setPrimitive(
        primitive.name, primitive.type, primitive.frame_id, primitive.visible,
        primitive.color, primitive.parameters);
    res->success.push_back(retval == 0 ? true : false);

    if (res->success.back()) {
      hiqp::printHiqpInfo("Set geometric primitive of type '" + primitive.type +
                          "' with name '" + primitive.name + "'.");
    }
  }
  return true;
}

bool HiQPServiceHandler::removePrimitives(
    const std::shared_ptr<hiqp_msgs::srv::RemovePrimitives::Request> req,
    std::shared_ptr<hiqp_msgs::srv::RemovePrimitives::Response> res){
  for (auto name : req->names) {
    if (task_manager_->removePrimitive(name) == 0)
      res->success.push_back(true);
    else
      res->success.push_back(false);

    if (res->success.back()) {
      hiqp::printHiqpInfo("Removed primitive '" + name + "' successfully!");
    } else {
      hiqp::printHiqpInfo("Couldn't remove primitive '" + name + "'!");
    }
  }
  return true;
}

bool HiQPServiceHandler::removeAllPrimitives(
    const std::shared_ptr<hiqp_msgs::srv::RemoveAllPrimitives::Request> req,
    std::shared_ptr<hiqp_msgs::srv::RemoveAllPrimitives::Response> res){
  task_manager_->removeAllPrimitives();
  hiqp::printHiqpInfo("Removed all primitives successfully!");
  res->success = true;
  return true;
}

bool HiQPServiceHandler::listAllPrimitives(
    const std::shared_ptr<hiqp_msgs::srv::ListAllPrimitives::Request> req,
    std::shared_ptr<hiqp_msgs::srv::ListAllPrimitives::Response> res){
  task_manager_->listAllPrimitives();
  res->success = true;
  return true;
}

bool HiQPServiceHandler::getAllPrimitives(
    const std::shared_ptr<hiqp_msgs::srv::GetAllPrimitives::Request> req,
    std::shared_ptr<hiqp_msgs::srv::GetAllPrimitives::Response> res){
  std::vector<hiqp::PrimitiveInfo> all_task_info =
      task_manager_->getAllPrimitiveInfo();
  for (auto it : all_task_info) {
    hiqp_msgs::msg::Primitive p;
    p.name = it.name;
    p.type = it.type;
    p.frame_id = it.frame_id;
    p.visible = it.visible;
    p.color = it.color;
    p.parameters = it.parameters;
    res->primitives.push_back(p);
  }
  return true;
}

bool HiQPServiceHandler::removePriorityLevel(
    const std::shared_ptr<hiqp_msgs::srv::RemovePriorityLevel::Request> req,
    std::shared_ptr<hiqp_msgs::srv::RemovePriorityLevel::Response> res){
  task_manager_->removePriorityLevel(req->priority);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::activatePriorityLevel(
    const std::shared_ptr<hiqp_msgs::srv::ActivatePriorityLevel::Request> req,
    std::shared_ptr<hiqp_msgs::srv::ActivatePriorityLevel::Response> res){
  task_manager_->activatePriorityLevel(req->priority);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::deactivatePriorityLevel(
    const std::shared_ptr<hiqp_msgs::srv::DeactivatePriorityLevel::Request> req,
    std::shared_ptr<hiqp_msgs::srv::DeactivatePriorityLevel::Response> res){
  task_manager_->deactivatePriorityLevel(req->priority);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::monitorPriorityLevel(
    const std::shared_ptr<hiqp_msgs::srv::MonitorPriorityLevel::Request> req,
    std::shared_ptr<hiqp_msgs::srv::MonitorPriorityLevel::Response> res){
  task_manager_->monitorPriorityLevel(req->priority);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::demonitorPriorityLevel(
    const std::shared_ptr<hiqp_msgs::srv::DemonitorPriorityLevel::Request> req,
    std::shared_ptr<hiqp_msgs::srv::DemonitorPriorityLevel::Response> res){
  task_manager_->demonitorPriorityLevel(req->priority);
  res->success = true;
  return true;
}

bool HiQPServiceHandler::isTaskSet(const std::shared_ptr<hiqp_msgs::srv::IsTaskSet::Request> req,
    std::shared_ptr<hiqp_msgs::srv::IsTaskSet::Response> res){
  res->is_set = task_manager_->isTaskSet(req->name);
  return true;
}
