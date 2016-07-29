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





cmake_minimum_required(VERSION 2.8.6)
project(hiqp_controllers)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
set(GUROBI_LIBS gurobi_c++ gurobi65)





find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  visualization_msgs
  kdl_parser
  hiqp_msgs_srvs
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CASADI REQUIRED)





catkin_package(
  CATKIN_DEPENDS roscpp controller_interface visualization_msgs kdl_parser hiqp_msgs_srvs
  INCLUDE_DIRS include #${EIGEN_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME} casadi
  DEPENDS orocos_kdl
)





include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CASADI_INCLUDE_DIR})
include_directories(${GUROBI_INCLUDE_DIR})

link_directories(${GUROBI_LIB_DIR})

add_library(${PROJECT_NAME} include/hiqp/hiqp_utils.h
                            include/hiqp/hiqp_kinematic_controller.h
                            include/hiqp/task_manager.h
                            include/hiqp/task_visualizer.h
                            include/hiqp/hiqp_solver.h
                            include/hiqp/casadi_solver.h
                            include/hiqp/task.h
                            include/hiqp/task_behaviour.h

                            include/hiqp/impl/task_geometric_projection.h
                            include/hiqp/impl/task_geometric_projection__impl.h
                            include/hiqp/impl/task_beh_fo.h

                            include/hiqp/geometric_primitives/geometric_primitive.h
                            include/hiqp/geometric_primitives/geometric_primitive_map.h
                            include/hiqp/geometric_primitives/geometric_point.h
                            include/hiqp/geometric_primitives/geometric_plane.h

                            src/hiqp_utils.cpp
                            src/hiqp_kinematic_controller.cpp
                            src/task_manager.cpp
                            src/task_visualizer.cpp
                            src/casadi_solver.cpp

                            src/impl/task_geometric_projection.cpp
                            src/impl/task_beh_fo.cpp

                            src/geometric_primitive_map.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${CASADI_LIBRARIES} ${GUROBI_LIBS})





install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES hiqp_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
