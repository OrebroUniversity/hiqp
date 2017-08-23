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

#include <limits>

#include <hiqp/utilities.h>

#include <hiqp/tasks/tdyn_pd.h>

namespace hiqp {
  namespace tasks {

    int TDynPD::init(const std::vector<std::string>& parameters, RobotStatePtr robot_state, const Eigen::VectorXd& e_initial, const Eigen::VectorXd& e_dot_initial, const Eigen::VectorXd& e_final, const Eigen::VectorXd& e_dot_final) {
      int dim = e_initial.rows();
      int size = parameters.size();
      assert((e_dot_initial.rows()==dim) && (e_final.rows() == dim) && (e_dot_final.rows() == dim) );
  
      if ((size != 2*dim*dim+1) && (size != 3)) {
	printHiqpWarning("TDynPD for a " + std::to_string(dim) + "-dimensional task requires either " + std::to_string(2*dim*dim+1) +" or 3 parameters, got " + std::to_string(size) + "! Initialization failed!");
	return -1;
      }
  
      //read the gain matrices - if only one P and D parameter is given, the Kp/Kd matrices are assumed to be diagonal with common gain
      if(size==3){
	Kp_=Eigen::MatrixXd::Identity(dim,dim)*std::stod(parameters.at(1));
	Kd_=Eigen::MatrixXd::Identity(dim,dim)*std::stod(parameters.at(2));
      }
      else{ //read the full matrices
	double* Kp_data = new double[dim*dim];
	double* Kd_data = new double[dim*dim];
	for (int i=0; i<dim*dim; i++){
	  Kp_data[i]=std::stod(parameters.at(i+1));
	  Kd_data[i]=stod(parameters.at(dim*dim+1+i));
	}
	Kp_ = (Eigen::Map<Eigen::MatrixXd>(Kp_data,dim,dim)).transpose();
	Kd_ = (Eigen::Map<Eigen::MatrixXd>(Kd_data,dim,dim)).transpose();
	delete Kp_data;
	delete Kd_data;
      }
      //assemble the corresponding dynamics matrix [0 I, -Kp_ -Kd_]
      Eigen::MatrixXd A(2*dim,2*dim);
      A.setZero();
      A.topRightCorner(dim,dim)=Eigen::MatrixXd::Identity(dim,dim);
      A.bottomLeftCorner(dim,dim)= -Kp_;
      A.bottomRightCorner(dim,dim)= -Kd_;
  
      //assert that the controller dynamics are stable
      Eigen::EigenSolver<Eigen::MatrixXd> es(A, false);
      if (es.eigenvalues().real().maxCoeff() >= 0.0) {
	printHiqpWarning("Unstable controller dynamics! Initialization failed!");
	return -1;
      }

      //initialize the controller
      e_ddot_star_= -Kp_*e_initial-Kd_*e_dot_initial;
      performance_measures_.resize(dim);

      // //=============Debug======================
      // std::cerr<<"e_initial: "<<e_initial.transpose()<<std::endl;
      // std::cerr<<"e_dot_initial: "<<e_dot_initial.transpose()<<std::endl;
      // std::cerr<<"e_final: "<<e_final.transpose()<<std::endl;
      // std::cerr<<"e_dot_final: "<<e_dot_final.transpose()<<std::endl;
      // std::cerr<<"dim: "<<dim<<std::endl;
      // std::cerr<<"size: "<<size<<std::endl;
      // std::cerr<<"Kp: "<<std::endl<<Kp_<<std::endl;
      // std::cerr<<"Kd: "<<std::endl<<Kd_<<std::endl;
      // std::cerr<<"A: "<<std::endl<<A<<std::endl;
      // std::cerr<<"Eigenvalues: "<<es.eigenvalues().transpose()<<std::endl;
      // std::cerr<<"e_ddot_star: "<<e_ddot_star_.transpose()<<std::endl;
      // // //===========End Debug====================
  
      return 0;
    }
    int TDynPD::update(const RobotStatePtr robot_state, const std::shared_ptr< TaskDefinition > def){
      //PD control law
      e_ddot_star_= -Kp_*def->getTaskValue()-Kd_*def->getTaskDerivative();
      //DEBUG===================================
      // std::cerr<<"e_ddot_star_: "<<e_ddot_star_.transpose()<<std::endl;
      // std::cerr<<".........................................."<<std::endl;
      //DEBUG END===============================
      return 0;
    }

    int TDynPD::monitor() { return 0; }

  }  // namespace tasks

}  // namespace hiqp
