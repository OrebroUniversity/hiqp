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

#include <hiqp/tasks/tdef_jnt_limits.h>
#include <hiqp/utilities.h>

#include <iostream>

namespace hiqp {
  namespace tasks {

    int TDefJntLimits::init(const std::vector<std::string>& parameters,
			    RobotStatePtr robot_state) {
      int size = parameters.size();
      if (size != 6) {
	printHiqpWarning("TDefJntLimits requires 6 parameter, got " +
			 std::to_string(size) + "! Initialization failed!");
	return -1;
      }

      link_frame_name_ = parameters.at(1);
      link_frame_q_nr_ =
	kdl_getQNrFromLinkName(robot_state->kdl_tree_, link_frame_name_);

      if (link_frame_q_nr_ < 0) {
	printHiqpWarning("TDefJntLimits::init, couldn't find joint '" +
			 link_frame_name_ + "'! Initialization failed.");
	return -2;
      }

      if (!robot_state->isQNrWritable(link_frame_q_nr_)) {
	printHiqpWarning("TDefJntLimits::init, the joint '" + link_frame_name_ +
			 "' is not writable! Initialization failed.");
	return -3;
      }

      ddq_max_= std::stod(parameters.at(2));
      dq_max_= std::stod(parameters.at(3));    
      q_lb_= std::stod(parameters.at(4));
      q_ub_= std::stod(parameters.at(5));
      
      assert(q_ub_ > q_lb_);
      assert(dq_max_ > 0.0);
      assert(ddq_max_ > 0.0);
      
      unsigned int n_joints = robot_state->getNumJoints(); 
      e_=Eigen::VectorXd::Zero(6);
      f_=Eigen::VectorXd::Zero(6);      
      e_dot_=Eigen::VectorXd::Zero(6);
      J_=Eigen::MatrixXd::Zero(6,n_joints);      
      J_dot_=Eigen::MatrixXd::Zero(6,n_joints);
      performance_measures_.resize(0);
      
      task_signs_.resize(6);
      task_signs_.at(0) = 1;   // > upper joint limit 
      task_signs_.at(1) = -1;  // < lower joint limit
      task_signs_.at(2) = -1;   // < upper joint velocity limit
      task_signs_.at(3) = 1;  // > lower joint velocity limit
      task_signs_.at(4) = -1;   // < upper joint acceleration limit
      task_signs_.at(5) = 1;  // > lower joint acceleration limit      

      J_(0,link_frame_q_nr_)=-1.0;      
      J_(1,link_frame_q_nr_)=-1.0;
      J_(2,link_frame_q_nr_)=1.0;
      J_(3,link_frame_q_nr_)=1.0;      
      J_(4,link_frame_q_nr_)=1.0;
      J_(5,link_frame_q_nr_)=1.0;

      //DEBUG ========================================
      // std::cerr<<"q_lb: "<<q_lb_<<std::endl;
      // std::cerr<<"q_ub: "<<q_ub_<<std::endl;
      // std::cerr<<"dq_max: "<<dq_max_<<std::endl;
      // std::cerr<<"ddq_max: "<<ddq_max_<<std::endl;
      // std::cerr<<"_____________________________________________________\n"<<std::endl;
      //DEBUG END ========================================

      return 0;
    }

    int TDefJntLimits::update(RobotStatePtr robot_state) {
      double q = robot_state->kdl_jnt_array_vel_.q(link_frame_q_nr_);
      double q_dot = robot_state->kdl_jnt_array_vel_.qdot(link_frame_q_nr_);

      e_(0)=q_ub_-q;
      e_dot_(0)=-q_dot;
      
      e_(1)=q_lb_-q;
      e_dot_(1)=-q_dot;
 
      e_(2)=dq_max_-q_dot;
      e_dot_(2)=0.0;

      e_(3)=-dq_max_-q_dot;
      e_dot_(3)=0.0;

      e_(4)=ddq_max_;
      e_dot_(4)=0.0;

      e_(5)= -ddq_max_;
      e_dot_(5)=0.0;
      
      //DEBUG===================================
      // std::cerr<<"link_frame_name_: "<<link_frame_name_<<std::endl;
      // std::cerr<<"link_frame_q_nr_: "<<link_frame_q_nr_<<std::endl;          std::cerr<<"q_lb_: "<<q_lb_<<std::endl;
      // std::cerr<<"q_ub_: "<<q_ub_<<std::endl;
      // std::cerr<<"dq_max_: "<<dq_max_<<std::endl;
      // std::cerr<<"q: "<<q<<std::endl;
      // std::cerr<<"q_dot: "<<q_dot<<std::endl;
      // std::cerr<<"e_: "<<e_.transpose()<<std::endl;
      // std::cerr<<"e_dot_: "<<e_dot_.transpose()<<std::endl;
      // std::cerr<<"J_: "<<std::endl<<J_<<std::endl;
      // std::cerr<<"(q_lb_ - q): "<<(q_lb_ - q)<<std::endl;
      // std::cerr<<"(q_ub_ - q): "<<(q_ub_ - q)<<std::endl;
      //DEBUG END===============================
      return 0;
    }

    int TDefJntLimits::monitor() { return 0; }

  }  // namespace tasks

}  // namespace hiqp
