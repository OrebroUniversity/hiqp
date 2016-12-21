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
  set_task_service_ = node_handle_->advertiseService(
    "set_task", &HiQPServiceHandler::setTask, this);

  remove_task_service_ = node_handle_->advertiseService(
    "remove_task", &HiQPServiceHandler::removeTask, this);

  remove_all_tasks_service_ = node_handle_->advertiseService(
    "remove_all_tasks", &HiQPServiceHandler::removeAllTasks, this);

  list_all_tasks_service_ = node_handle_->advertiseService(
    "list_all_tasks", &HiQPServiceHandler::listAllTasks, this);

  add_primitive_service_ = node_handle_->advertiseService(
    "add_primitive", &HiQPServiceHandler::addPrimitive, this);

  remove_primitive_service_ = node_handle_->advertiseService(
    "remove_primitive", &HiQPServiceHandler::removePrimitive, this);

  remove_all_primitives_service_ = node_handle_->advertiseService(
    "remove_all_primitives", &HiQPServiceHandler::removeAllPrimitives, this);

  list_all_primitives_service_ = node_handle_->advertiseService(
    "list_all_primitives", &HiQPServiceHandler::listAllPrimitives, this);
}

bool HiQPServiceHandler::setTask(hiqp_msgs::SetTask::Request& req, 
                                 hiqp_msgs::SetTask::Response& res) {
  int retval = task_manager_->setTask(
    req.name, req.priority, req.visible, req.active,
    req.def_params, req.dyn_params, robot_state_);
  res.success = (retval < 0 ? false : true);
  return true;
}

bool HiQPServiceHandler::removeTask(hiqp_msgs::RemoveTask::Request& req, 
                                    hiqp_msgs::RemoveTask::Response& res) {
  res.success = false;
  if (task_manager_->removeTask(req.task_name) == 0)
    res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed task '" + req.task_name + "'.");
  } else {
    hiqp::printHiqpInfo("Couldn't remove task '" + req.task_name + "'!");  
  }
  return true;
}

bool HiQPServiceHandler::removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, 
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

bool HiQPServiceHandler::addPrimitive(hiqp_msgs::AddPrimitive::Request& req, 
                                      hiqp_msgs::AddPrimitive::Response& res) {
  int retval = task_manager_->addPrimitive(
    req.name, req.type, req.frame_id, req.visible, req.color, req.parameters
  );
  res.success = (retval == 0 ? true : false);
  if (res.success) {
    hiqp::printHiqpInfo("Added geometric primitive of type '" + req.type + "' with name '" + req.name + "'.");
  }
  return true;
}

bool HiQPServiceHandler::removePrimitive(hiqp_msgs::RemovePrimitive::Request& req, 
                                         hiqp_msgs::RemovePrimitive::Response& res) {
  res.success = false;
  if (task_manager_->removePrimitive(req.name) == 0)
    res.success = true;

  if (res.success) {
    hiqp::printHiqpInfo("Removed primitive '" + req.name + "' successfully!");
  } else {
    hiqp::printHiqpInfo("Couldn't remove primitive '" + req.name + "'!");  
  }
  return true;
}

bool HiQPServiceHandler::removeAllPrimitives(hiqp_msgs::RemoveAllPrimitives::Request& req, 
                                             hiqp_msgs::RemoveAllPrimitives::Response& res) {
  task_manager_->removeAllPrimitives();
  hiqp::printHiqpInfo("Removed all primitives successfully!");
  res.success = true;
  return true;
}

bool HiQPServiceHandler::listAllPrimitives(hiqp_msgs::ListAllPrimitives::Request& req, 
                                           hiqp_msgs::ListAllPrimitives::Response& res) {
  task_manager_->listAllPrimitives();
  res.success = true;
  return true;
}