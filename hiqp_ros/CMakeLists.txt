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
#set(CMAKE_BUILD_TYPE Debug)
  
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
pluginlib
generate_parameter_library
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
    hiqp_parameters
    src/hiqp_parameters.yaml
    )

set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include")
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
set(GUROBI_LIBS gurobi_c++ gurobi100)

#------------------------------- Targets ------------------------------#

# hiqp-client library and executale
add_library(hiqp_client SHARED src/hiqp_client.cpp)
target_include_directories(
  hiqp_client
  PUBLIC
  ${GUROBI_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_directories(hiqp_client PUBLIC ${GUROBI_LIB_DIR})
target_link_libraries(hiqp_client PUBLIC ${GUROBI_LIBS})
# hiqp_client_test
add_executable (test_hiqp_client src/test_hiqp_client.cpp)
target_link_libraries(test_hiqp_client PUBLIC hiqp_client)
ament_target_dependencies(hiqp_client PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
#----------------------------------------------------------------------#

# hiqp-ros library
add_library(${PROJECT_NAME} SHARED 
                            src/ros_topic_subscriber.cpp
                            src/ros_visualizer.cpp
                            src/hiqp_service_handler.cpp
                            src/hiqp_controller.cpp
                            src/utilities.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${GUROBI_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_directories(${PROJECT_NAME} PUBLIC ${GUROBI_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC hiqp_parameters ${GUROBI_LIBS})
pluginlib_export_plugin_description_file(controller_interface hiqp_plugin.xml)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
#----------------------------------------------------------------------#

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

#===================================================================================================
# Installation
#===================================================================================================
ament_export_targets(export_hiqp_ros HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
install(TARGETS hiqp_client hiqp_parameters ${PROJECT_NAME}
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
