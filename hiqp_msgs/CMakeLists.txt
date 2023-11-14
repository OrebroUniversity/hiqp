# The HiQP Control Framework, an optimal control framework targeted at robotics
# Copyright (C) 2016 Marcus A Johansson
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.8)
project(hiqp_msgs)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs      REQUIRED)

set(msg_files
    "msg/TaskMeasure.msg"
    "msg/Primitive.msg"
    "msg/TaskMeasures.msg"
    "msg/StringArray.msg"
    "msg/JointControllerState.msg"
    "msg/JointInfo.msg"
    "msg/FTSInfo.msg"
    "msg/Vector3d.msg"
    "msg/Task.msg"
)

set(srv_files 
    "srv/SetTasks.srv"
    "srv/RemoveTasks.srv"
    "srv/RemoveAllTasks.srv"
    "srv/ListAllTasks.srv"
    "srv/GetAllTasks.srv"
    "srv/ActivateTask.srv"
    "srv/DeactivateTask.srv"
    "srv/MonitorTask.srv"
    "srv/DemonitorTask.srv"
    "srv/SetPrimitives.srv"
    "srv/RemovePrimitives.srv"
    "srv/RemoveAllPrimitives.srv"
    "srv/ListAllPrimitives.srv"
    "srv/GetAllPrimitives.srv"
    "srv/RemovePriorityLevel.srv"
    "srv/ActivatePriorityLevel.srv"
    "srv/DeactivatePriorityLevel.srv"
    "srv/MonitorPriorityLevel.srv"
    "srv/DemonitorPriorityLevel.srv"
    "srv/IsTaskSet.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces visualization_msgs geometry_msgs std_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
