cmake_minimum_required(VERSION 2.8.3)
project(irb4400_description)

find_package(catkin REQUIRED COMPONENTS roslaunch roscpp rospy std_msgs genmsg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(irb4400 src/irb4400.cpp)
#target_link_libraries(irb4400 ${catkin_LIBRARIES})
#add_dependencies(irb4400 reload_generate_messages_cpp)

roslaunch_add_file_check(launch)

install(DIRECTORY config images launch rviz urdf meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
