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

#ifndef HIQP_TDYN_PD_H
#define HIQP_TDYN_PD_H

#include <hiqp/robot_state.h>
#include <hiqp/task_dynamics.h>

namespace hiqp {
  namespace tasks {

    /*! \brief A general second-order task dynamics (i.e., a PD controller) implementation of the form e_ddot= -Kp*e-Kd*e_dot, where e, e_dot, e_ddot are in R^m, and Kp, Kd are in R^{m x m}. The controller needs to be stable, i. e., [0 I, -Kp_ -Kd_] needs to be negative definite. Note that in order to avoid overshooting, the resulting system needs to be overdamped, i. e., K_d >= 2*sqrt(K_p)
     *  \author Marcus A Johansson, Robert Krug */
    class TDynPD : public TaskDynamics {
    public:
    TDynPD(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
	   std::shared_ptr<Visualizer> visualizer)
      : TaskDynamics(geom_prim_map, visualizer) {}

      ~TDynPD() noexcept {}

      /*! \brief It is assumed that the two gain matrices Kp & Kd in R^{m x m} are given subsequently in row-major format in the parameter string, i. e., Kp_ = parameters[1] - parameters[m^2], and Kd_ = parameters[m^2+1] - parameters[2*m^2], where m is the task space dimension
       */
      int init(const std::vector<std::string>& parameters, RobotStatePtr robot_state, const Eigen::VectorXd& e_initial, const Eigen::VectorXd& e_dot_initial, const Eigen::VectorXd& e_final, const Eigen::VectorXd& e_dot_final);

      int update(const RobotStatePtr robot_state, const TaskDefinitionPtr def);
      int monitor();
    protected:
      Eigen::MatrixXd Kp_; ///< Controller proportional gain matrix
      Eigen::MatrixXd Kd_; ///< Controller derivative gain matrix
      
    private:
      TDynPD(const TDynPD& other) = delete;
      TDynPD(TDynPD&& other) = delete;
      TDynPD& operator=(const TDynPD& other) = delete;
      TDynPD& operator=(TDynPD&& other) noexcept = delete;


    };

  }  // namespace tasks

}  // namespace hiqp

#endif  // include guard
