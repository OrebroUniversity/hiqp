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

#include <hiqp/utilities.h>

#include <hiqp/tasks/tdyn_jnt_limits.h>

namespace hiqp {
  namespace tasks {

    int TDynJntLimits::init(const std::vector<std::string>& parameters, RobotStatePtr robot_state, const Eigen::VectorXd& e_initial, const Eigen::VectorXd& e_dot_initial, const Eigen::VectorXd& e_final, const Eigen::VectorXd& e_dot_final) {
      assert((e_dot_initial.rows()==6) && (e_final.rows() == 6) && (e_dot_final.rows() == 6) );
      int size = parameters.size();
      if (size != 3) {
	printHiqpWarning("TDynJntLimits requires 3 parameters, got " +
			 std::to_string(size) + "! Initialization failed!");
	return -1;
      }
      Kp_=std::stod(parameters.at(1)); //P gain for joint limits
      Kd_=std::stod(parameters.at(2)); //D gain for joint limits

      performance_measures_.resize(0);
      e_ddot_star_.resize(6);
      double dt=robot_state->sampling_time_;
   
      e_ddot_star_(0)=-Kp_*e_initial(0)-Kd_*e_dot_initial(0);
      e_ddot_star_(1)=-Kp_*e_initial(1)-Kd_*e_dot_initial(1);
      e_ddot_star_(2)=1/dt*e_initial(2);
      e_ddot_star_(3)=1/dt*e_initial(3);
      e_ddot_star_(4)=e_initial(4);
      e_ddot_star_(5)=e_initial(5);

      //Truncate the desired accelerations from the joint limit avoidance in order to avoid possible infeasibilites 
      for(unsigned int i=0; i<4; i++){
	if(e_ddot_star_(i) > e_initial(4)) e_ddot_star_(i)=e_initial(4);
	if(e_ddot_star_(i) < e_initial(5)) e_ddot_star_(i)=e_initial(5);
      }
  
      // =============Debug======================
      // std::cerr<<"e_initial: "<<e_initial.transpose()<<std::endl;
      // std::cerr<<"e_dot_initial: "<<e_dot_initial.transpose()<<std::endl;
      // std::cerr<<"e_final: "<<e_final.transpose()<<std::endl;
      // std::cerr<<"e_dot_final: "<<e_dot_final.transpose()<<std::endl;
      // std::cerr<<"size: "<<size<<std::endl;
      // std::cerr<<"Kp_: "<<std::endl<<Kp_<<std::endl;
      // std::cerr<<"Kd_: "<<std::endl<<Kd_<<std::endl;
      // std::cerr<<"e_ddot_star: "<<e_ddot_star_.transpose()<<std::endl;
      // ===========End Debug====================

      return 0;
    }

    int TDynJntLimits::update(const RobotStatePtr robot_state, const std::shared_ptr< TaskDefinition > def){

      double dt=robot_state->sampling_time_;
      Eigen::VectorXd e=def->getTaskValue();
      Eigen::VectorXd e_dot=def->getTaskDerivative();   
   
      e_ddot_star_(0)=-Kp_*e(0)-Kd_*e_dot(0);
      e_ddot_star_(1)=-Kp_*e(1)-Kd_*e_dot(1);
      e_ddot_star_(2)=1/dt*e(2);
      e_ddot_star_(3)=1/dt*e(3);
      e_ddot_star_(4)=e(4);
      e_ddot_star_(5)=e(5);

      //Truncate the desired accelerations from the joint limit avoidance in order to avoid possible infeasibilites 
      for(unsigned int i=0; i<4; i++){
	if(e_ddot_star_(i) > e(4)) e_ddot_star_(i)=e(4);
	if(e_ddot_star_(i) < e(5)) e_ddot_star_(i)=e(5);
      }
   
      //DEBUG===================================
      //  std::cerr<<"dt: "<<dt<<std::endl;
      // std::cerr<<"e_ddot_star_: "<<e_ddot_star_.transpose()<<std::endl;
      //DEBUG END ===============================
  
      return 0;
    }

    int TDynJntLimits::monitor() { return 0; }

  }  // namespace tasks

}  // namespace hiqp
