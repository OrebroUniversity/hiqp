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

#include <hiqp_msgs/SetTask.h>
#include <hiqp_msgs/UpdateTask.h>
#include <hiqp_msgs/RemoveTask.h>
#include <hiqp_msgs/RemoveAllTasks.h>
#include <hiqp_msgs/ListAllTasks.h>
#include <hiqp_msgs/AddGeometricPrimitive.h>
#include <hiqp_msgs/RemoveGeometricPrimitive.h>
#include <hiqp_msgs/RemoveAllGeometricPrimitives.h>
#include <hiqp_msgs/ListAllGeometricPrimitives.h>

class HiQPServiceHandler {
public:
  HiQPServiceHandler();
  ~HiQPServiceHandler();

private:
  HiQPServiceHandler(const HiQPServiceHandler& other) = delete;
    HiQPServiceHandler(HiQPServiceHandler&& other) = delete;
    HiQPServiceHandler& operator=(const HiQPServiceHandler& other) = delete;
    HiQPServiceHandler& operator=(HiQPServiceHandler&& other) noexcept = delete;

    bool setTask(hiqp_msgs::SetTask::Request& req, hiqp_msgs::SetTask::Response& res);
    bool removeTask(hiqp_msgs::RemoveTask::Request& req, hiqp_msgs::RemoveTask::Response& res);
    bool removeAllTasks(hiqp_msgs::RemoveAllTasks::Request& req, hiqp_msgs::RemoveAllTasks::Response& res);
    bool listAllTasks(hiqp_msgs::ListAllTasks::Request& req, hiqp_msgs::ListAllTasks::Response& res);
    bool addGeometricPrimitive(hiqp_msgs::AddGeometricPrimitive::Request& req, hiqp_msgs::AddGeometricPrimitive::Response& res);
    bool removeGeometricPrimitive(hiqp_msgs::RemoveGeometricPrimitive::Request& req, hiqp_msgs::RemoveGeometricPrimitive::Response& res);
    bool removeAllGeometricPrimitives(hiqp_msgs::RemoveAllGeometricPrimitives::Request& req, hiqp_msgs::RemoveAllGeometricPrimitives::Response& res);
    bool listAllGeometricPrimitives(hiqp_msgs::ListAllGeometricPrimitives::Request& req, hiqp_msgs::ListAllGeometricPrimitives::Response& res);

    ros::ServiceServer    set_task_service_;
    ros::ServiceServer    update_task_service_;
    ros::ServiceServer    remove_task_service_;
    ros::ServiceServer    remove_all_tasks_service_;
    ros::ServiceServer    list_all_tasks_service_;
    ros::ServiceServer    add_geomprim_service_;
    ros::ServiceServer    remove_geomprim_service_;
    ros::ServiceServer    remove_all_geomprims_service_;
    ros::ServiceServer    list_all_geomprims_service_;
};

#endif