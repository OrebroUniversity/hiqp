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
project(hiqp_ros)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Wl,-z,defs)
endif()

set(CMAKE_VERBOSE_MAKEFILE off)

find_package(ament_cmake          REQUIRED)
set(THIS_PACKAGE_INCLUDE_DEPENDS
rclcpp              
orocos_kdl          
controller_interface
visualization_msgs  
kdl_parser          
hiqp_core           
realtime_tools			
hiqp_msgs           
control_msgs       
tf2_msgs
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include")
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
set(GUROBI_LIBS gurobi_c++ gurobi100)

include_directories(
  ${GUROBI_INCLUDE_DIR}
)
link_directories(${GUROBI_LIB_DIR})

#------------------------------- Targets ------------------------------#

# hiqp-client library
add_library(hiqp_client src/hiqp_client.cpp)
target_include_directories(
  hiqp_client
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(hiqp_client ${GUROBI_LIBS})

# hiqp_client_test
add_executable (test_hiqp_client src/test_hiqp_client.cpp)
target_link_libraries(test_hiqp_client hiqp_client)


# hiqp-ros library
add_library(${PROJECT_NAME} 
                            src/ros_topic_subscriber.cpp
                            src/utilities.cpp)
#    src/hiqp_joint_velocity_controller.cpp
#                            src/hiqp_joint_effort_controller.cpp
#                            src/hiqp_service_handler.cpp
#                            src/ros_topic_subscriber.cpp
#                            src/ros_visualizer.cpp

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} ${GUROBI_LIBS})

#add_dependencies(${PROJECT_NAME} hiqp_msgs_generate_messages_cpp)

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GUROBI_LIBS})

#add_dependencies(hiqp_client hiqp_msgs_generate_messages_cpp)


# TODO needs to be ported again if anyone cares for it?
# hiqp-joint-trajectory-controller
#add_executable(hiqp_joint_trajectory_controller_node src/hiqp_joint_trajectory_controller.cpp)
#add_dependencies(hiqp_joint_trajectory_controller_node ${catkin_EXPORTED_TARGETS})
#target_link_libraries(hiqp_joint_trajectory_controller_node hiqp_client ${catkin_LIBRARIES} ${GUROBI_LIBS})


#add_dependencies(test_hiqp_client hiqp_msgs_generate_messages_cpp)
#---------------------------Dependencies ------------------------#
ament_target_dependencies(hiqp_client ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_hiqp_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
#===================================================================================================
# Installation
#===================================================================================================
install(TARGETS hiqp_client
        EXPORT export_hiqp_ros
        ARCHIVE DESTINATION lib 
        LIBRARY DESTINATION lib 
        RUNTIME DESTINATION bin)

install(
  DIRECTORY "include/"
  DESTINATION include
)

#install(DIRECTORY include/${PROJECT_NAME}/
#        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        FILES_MATCHING PATTERN "*.h")

#install(TARGETS ${PROJECT_NAME}
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(FILES plugins.xml
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
