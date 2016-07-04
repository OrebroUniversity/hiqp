cmake_minimum_required(VERSION 2.8.3)
project(hiqp_msgs_srvs)

## Author: Marcus A Johansson (marcus.adam.johansson@gmail.com)
## Date: July, 2016


find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs
  visualization_msgs
  geometry_msgs)



add_message_files(
  FILES
  TaskSpec.msg
  )



add_service_files(
  FILES
  AddTask.srv
  )



generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  visualization_msgs
  )



catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs 
  visualization_msgs
  geometry_msgs
  )

