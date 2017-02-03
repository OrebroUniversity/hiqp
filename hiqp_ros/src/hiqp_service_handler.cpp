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

void HiQPServiceHandler::advertiseAll() {
  set_tasks_service_ = node_handle_->advertiseService(
      "set_tasks", &HiQPServiceHandler::setTasks, this);
  remove_task_service_ = node_handle_->advertiseService(
      "remove_task", &HiQPServiceHandler::removeTask, this);
  remove_all_tasks_service_ = node_handle_->advertiseService(
      "remove_all_tasks", &HiQPServiceHandler::removeAllTasks, this);
  list_all_tasks_service_ = node_handle_->advertiseService(
      "list_all_tasks", &HiQPServiceHandler::listAllTasks, this);
  get_all_tasks_service_ = node_handle_->advertiseService(
      "get_all_tasks", &HiQPServiceHandler::getAllTasks, this);
  activate_task_service_ = node_handle_->advertiseService(
      "activate_task", &HiQPServiceHandler::activateTask, this);
  deactivate_task_service_ = node_handle_->advertiseService(
      "deactivate_task", &HiQPServiceHandler::deactivateTask, this);
  monitor_task_service_ = node_handle_->advertiseService(
      "monitor_task", &HiQPServiceHandler::monitorTask, this);
  demonitor_task_service_ = node_handle_->advertiseService(
      "demonitor_task", &HiQPServiceHandler::demonitorTask, this);

  set_primitives_service_ = node_handle_->advertiseService(
      "set_primitives", &HiQPServiceHandler::setPrimitives, this);
  remove_primitive_service_ = node_handle_->advertiseService(
      "remove_primitive", &HiQPServiceHandler::removePrimitive, this);
  remove_all_primitives_service_ = node_handle_->advertiseService(
      "remove_all_primitives", &HiQPServiceHandler::removeAllPrimitives, this);
  list_all_primitives_service_ = node_handle_->advertiseService(
      "list_all_primitives", &HiQPServiceHandler::listAllPrimitives, this);
  get_all_primitives_service_ = node_handle_->advertiseService(
      "get_all_primitives", &HiQPServiceHandler::getAllPrimitives, this);

  remove_priority_level_service_ = node_handle_->advertiseService(
      "remove_priority_level", &HiQPServiceHandler::removePriorityLevel, this);
  activate_priority_level_service_ = node_handle_->advertiseService(
      "activate_priority_level", &HiQPServiceHandler::activatePriorityLevel,
      this);
  deactivate_priority_level_service_ = node_handle_->advertiseService(
      "deactivate_priority_level", &HiQPServiceHandler::deactivatePriorityLevel,
      this);
  monitor_priority_level_service_ = node_handle_->advertiseService(
      "monitor_priority_level", &HiQPServiceHandler::monitorPriorityLevel,
      this);
  demonitor_priority_level_service_ = node_handle_->advertiseService(
      "demonitor_priority_level", &HiQPServiceHandler::demonitorPriorityLevel,
      this);
}

bool HiQPServiceHandler::setTasks(hiqp_msgs::SetTasks::Request& req,
                                  hiqp_msgs::SetTasks::Response& res) {
  for (auto task : req.tasks) {
    int retval = task_manager_->setTask(
        task.name, task.priority, task.visible, task.active, task.monitored,
        task.def_params, task.dyn_params, robot_state_);
    res.success.push_back(retval < 0 ? false : true);
  }
  return true;
}

bool HiQPServiceHandler::removeTask(hiqp_msgs::RemoveTask::Request& req,
                                    hiqp_msgs::RemoveTask::Response& res) {
  res.success = false;
  if (task_manager_->removeTask(req.name) == 0) res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed task '" + req.name + "'.");
  } else {
    hiqp::printHiqpInfo("Couldn't remove task '" + req.name + "'!");
  }
  return true;
}

bool HiQPServiceHandler::removeAllTasks(
    hiqp_msgs::RemoveAllTasks::Request& req,
    hiqp_msgs::RemoveAllTasks::Response& res) {
  task_manager_->removeAllTasks();
  hiqp::printHiqpInfo("Removed all tasks successfully!");
  res.success = true;
  return true;
}

bool HiQPServiceHandler::listAllTasks(hiqp_msgs::ListAllTasks::Request& req,
                                      hiqp_msgs::ListAllTasks::Response& res) {
  task_manager_->listAllTasks();
  res.success = true;
  return true;
}

bool HiQPServiceHandler::getAllTasks(hiqp_msgs::GetAllTasks::Request& req,
                                     hiqp_msgs::GetAllTasks::Response& res) {
  std::vector<hiqp::TaskInfo> all_task_info = task_manager_->getAllTaskInfo();
  for (auto it : all_task_info) {
    hiqp_msgs::Task t;
    t.name = it.name;
    t.priority = it.priority;
    t.active = it.active;
    t.monitored = it.monitored;
    t.def_params = it.def_params;
    t.dyn_params = it.dyn_params;
    res.tasks.push_back(t);
  }
  return true;
}

bool HiQPServiceHandler::activateTask(hiqp_msgs::ActivateTask::Request& req,
                                      hiqp_msgs::ActivateTask::Response& res) {
  task_manager_->activateTask(req.name);
  res.success = true;
  return true;
}
bool HiQPServiceHandler::deactivateTask(
    hiqp_msgs::DeactivateTask::Request& req,
    hiqp_msgs::DeactivateTask::Response& res) {
  task_manager_->deactivateTask(req.name);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::monitorTask(hiqp_msgs::MonitorTask::Request& req,
                                     hiqp_msgs::MonitorTask::Response& res) {
  task_manager_->monitorTask(req.name);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::demonitorTask(
    hiqp_msgs::DemonitorTask::Request& req,
    hiqp_msgs::DemonitorTask::Response& res) {
  task_manager_->demonitorTask(req.name);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::setPrimitives(
    hiqp_msgs::SetPrimitives::Request& req,
    hiqp_msgs::SetPrimitives::Response& res) {
  for (auto primitive : req.primitives) {
    int retval = task_manager_->setPrimitive(
        primitive.name, primitive.type, primitive.frame_id, primitive.visible,
        primitive.color, primitive.parameters);
    res.success.push_back(retval == 0 ? true : false);

    if (res.success.back()) {
      hiqp::printHiqpInfo("Set geometric primitive of type '" + primitive.type +
                          "' with name '" + primitive.name + "'.");
    }
  }
  return true;
}

bool HiQPServiceHandler::removePrimitive(
    hiqp_msgs::RemovePrimitive::Request& req,
    hiqp_msgs::RemovePrimitive::Response& res) {
  res.success = false;
  if (task_manager_->removePrimitive(req.name) == 0) res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed primitive '" + req.name + "' successfully!");
  } else {
    hiqp::printHiqpInfo("Couldn't remove primitive '" + req.name + "'!");
  }
  return true;
}

bool HiQPServiceHandler::removeAllPrimitives(
    hiqp_msgs::RemoveAllPrimitives::Request& req,
    hiqp_msgs::RemoveAllPrimitives::Response& res) {
  task_manager_->removeAllPrimitives();
  hiqp::printHiqpInfo("Removed all primitives successfully!");
  res.success = true;
  return true;
}

bool HiQPServiceHandler::listAllPrimitives(
    hiqp_msgs::ListAllPrimitives::Request& req,
    hiqp_msgs::ListAllPrimitives::Response& res) {
  task_manager_->listAllPrimitives();
  res.success = true;
  return true;
}

bool HiQPServiceHandler::getAllPrimitives(
    hiqp_msgs::GetAllPrimitives::Request& req,
    hiqp_msgs::GetAllPrimitives::Response& res) {
  std::vector<hiqp::PrimitiveInfo> all_task_info =
      task_manager_->getAllPrimitiveInfo();
  for (auto it : all_task_info) {
    hiqp_msgs::Primitive p;
    p.name = it.name;
    p.type = it.type;
    p.frame_id = it.frame_id;
    p.visible = it.visible;
    p.color = it.color;
    p.parameters = it.parameters;
    res.primitives.push_back(p);
  }
  return true;
}

bool HiQPServiceHandler::removePriorityLevel(
    hiqp_msgs::RemovePriorityLevel::Request& req,
    hiqp_msgs::RemovePriorityLevel::Response& res) {
  task_manager_->removePriorityLevel(req.priority);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::activatePriorityLevel(
    hiqp_msgs::ActivatePriorityLevel::Request& req,
    hiqp_msgs::ActivatePriorityLevel::Response& res) {
  task_manager_->activatePriorityLevel(req.priority);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::deactivatePriorityLevel(
    hiqp_msgs::DeactivatePriorityLevel::Request& req,
    hiqp_msgs::DeactivatePriorityLevel::Response& res) {
  task_manager_->deactivatePriorityLevel(req.priority);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::monitorPriorityLevel(
    hiqp_msgs::MonitorPriorityLevel::Request& req,
    hiqp_msgs::MonitorPriorityLevel::Response& res) {
  task_manager_->monitorPriorityLevel(req.priority);
  res.success = true;
  return true;
}

bool HiQPServiceHandler::demonitorPriorityLevel(
    hiqp_msgs::DemonitorPriorityLevel::Request& req,
    hiqp_msgs::DemonitorPriorityLevel::Response& res) {
  task_manager_->demonitorPriorityLevel(req.priority);
  res.success = true;
  return true;
}
