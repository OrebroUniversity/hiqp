# The HiQP Control Framework, an optimal control framework targeted at robotics
# Copyright (C) 2016 Marcus A Johansson
# Copyright (C) 2023 Todor Stoyanov
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
project(hiqp_core)

# Available backends: gurobi (stable), rpsolver (unstable)
set(HIQP_QPSOLVER_BACKEND "gurobi")
#set(HIQP_QPSOLVER_BACKEND "rpsolver")

set(CMAKE_BUILD_TYPE RelWithDebInfo)
  
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17 -Wl,-z,defs)
endif()

set(CMAKE_VERBOSE_MAKEFILE off)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rcpputils 
  pluginlib 
  orocos_kdl
  Eigen3 
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

set(SOLVER_SOURCE_FILE "")
if(${HIQP_QPSOLVER_BACKEND} STREQUAL "gurobi")
set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")

# ACQUIRE GUROBI VERSION NUMBER
file(GLOB GUROBI_LIB_FILES "${GUROBI_LIB_DIR}/libgurobi*.so")
if(GUROBI_LIB_FILES)  # if any matching files were found
    list(GET GUROBI_LIB_FILES 0 GUROBI_FIRST_LIB)
    string(REGEX MATCH "libgurobi([0-9]+)\\.so" GUROBI_VERSION_MATCH ${GUROBI_FIRST_LIB})
    if(GUROBI_VERSION_MATCH)  # Check if a version number was found
        set(GUROBI_VERSION ${CMAKE_MATCH_1})
    else()
        message(FATAL_ERROR "Unable to infer Gurobi version number from the filename.")
    endif()
else()
    message(FATAL_ERROR "No Gurobi library files found in the specified directory ${GUROBI_LIB_DIR}")
endif()

set(GUROBI_LIBS gurobi_c++ gurobi${GUROBI_VERSION})
include_directories(${GUROBI_INCLUDE_DIR})
link_directories(${GUROBI_LIB_DIR})
set(SOLVER_SOURCE_FILE "src/solvers/gurobi_solver.cpp" "src/solvers/gurobi_solver_cascade.cpp")
add_definitions(-DHIQP_GUROBI)
elseif(${HIQP_QPSOLVER_BACKEND} STREQUAL "rpsolver")
set(SOLVER_SOURCE_FILE "src/solvers/rp_solver.cpp")
add_definitions(-DHIQP_RP_SOLVER)
endif()

add_library(${PROJECT_NAME} SHARED 
        src/utilities.cpp
		    src/hiqp_time_point.cpp
		    src/task_manager.cpp
		    src/task.cpp
		    #src/geometric_primitives/geometric_primitive.cpp
		    src/geometric_primitives/geometric_primitive_map.cpp
		    ${SOLVER_SOURCE_FILE}
        src/tasks/tdyn_pd.cpp
        #src/tasks/tdyn_cubic.cpp
        #src/tasks/tdyn_hyper_sin.cpp
        src/tasks/tdyn_jnt_limits.cpp
        #src/tasks/tdyn_minimal_jerk.cpp
        src/tasks/tdyn_linear_impedance.cpp
        src/tasks/tdyn_linear.cpp
        #src/tasks/tdef_meta_task.cpp
        src/tasks/tdef_tracking.cpp
        src/tasks/tdef_full_pose.cpp
        src/tasks/tdef_geometric_alignment.cpp
        src/tasks/tdef_geometric_projection.cpp
        src/tasks/tdef_fts_projection.cpp			    
        src/tasks/tdef_jnt_config.cpp
        src/tasks/tdef_jnt_limits.cpp
			    )

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(${HIQP_QPSOLVER_BACKEND} STREQUAL "gurobi")
  target_link_libraries(${PROJECT_NAME} PUBLIC ${orocos_kdl_LIBRARIES} ${GUROBI_LIBS})
else()
  target_link_libraries(${PROJECT_NAME} PUBLIC ${orocos_kdl_LIBRARIES})
endif()

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_targets(export_hiqp_core HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()

#----------------------- Install --------------------#
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_hiqp_core
        ARCHIVE DESTINATION lib 
        LIBRARY DESTINATION lib 
        RUNTIME DESTINATION bin)
