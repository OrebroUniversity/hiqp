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

#include <hiqp/tasks/tdef_jnt_config.h>
#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp {
namespace tasks {

int TDefJntConfig::init(const std::vector<std::string>& parameters,
                        RobotStatePtr robot_state) {
  int size = parameters.size();
  if (size != 3) {
    printHiqpWarning("TDefJntConfig requires 3 parameters, got " +
                     std::to_string(size) + "! Initialization failed!");
    return -1;
  }

  link_name_ = parameters.at(1);

  joint_q_nr_ = kdl_getQNrFromLinkName(robot_state->kdl_tree_, link_name_);

  if (joint_q_nr_ < 0) {
    printHiqpWarning("TDefJntConfig::init, couldn't find joint '" + link_name_ +
                     "'! Initialization failed.");
    return -2;
  }

  if (!robot_state->isQNrWritable(joint_q_nr_)) {
    printHiqpWarning("TDefJntConfig::init, the joint '" + link_name_ +
                     "' is not writable! Initialization failed.");
    return -3;
  }

  desired_configuration_ = std::stod(parameters.at(2));

  task_signs_.insert(task_signs_.begin(), 1, 0); //equality task
  
  unsigned int n_joints = robot_state->getNumJoints();
  f_=Eigen::VectorXd::Zero(1);
  e_=Eigen::VectorXd::Zero(1);
  e_dot_=Eigen::VectorXd::Zero(1);
  J_=Eigen::MatrixXd::Zero(1,n_joints);      
  J_dot_=Eigen::MatrixXd::Zero(1,n_joints);
  performance_measures_.resize(0);    

  //Jacobian is constant 
  J_(0, joint_q_nr_) = -1;

  return 0;
}

  int TDefJntConfig::update(RobotStatePtr robot_state) {
    const KDL::JntArray& q = robot_state->kdl_jnt_array_vel_.q;
    const KDL::JntArray& q_dot = robot_state->kdl_jnt_array_vel_.qdot;
  
    e_(0) = desired_configuration_ - q(joint_q_nr_);
    e_dot_(0)= -q_dot(joint_q_nr_);

    //DEBUG===================================
      // if(joint_q_nr_ == 0){
      // std::cerr<<"link_name_: "<<link_name_<<std::endl;
      // std::cerr<<"joint_q_nr_: "<<joint_q_nr_<<std::endl;
      //       std::cerr<<"e_: "<<e_.transpose()<<std::endl;
      // std::cerr<<"e_dot_: "<<e_dot_.transpose()<<std::endl;
      //       }
      //DEBUG END===============================

  
  return 0;
}

int TDefJntConfig::monitor() { return 0; }

}  // namespace tasks

}  // namespace hiqp
