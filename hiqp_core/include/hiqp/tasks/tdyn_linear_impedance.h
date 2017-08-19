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

#ifndef HIQP_TDYN_LIN_IMP_H
#define HIQP_TDYN_LIN_IMP_H

#include <hiqp/robot_state.h>
#include <hiqp/tasks/tdyn_pd.h>

namespace hiqp {
  namespace tasks {

    /*! \brief A second-order linear impedance controller a (i.e., a PD controller with an exogeneous force term) implementation of the form e_ddot= B^(-1)(-Kp*e-Kd*e_dot+f), where e, e_dot, e_ddot and f are in R^m, and Kp, Kd are in R^{m x m}. The controller needs to be stable, i. e., [0 I, -Kp_ -Kd_] needs to be negative definite. Note that in order to avoid overshooting, the resulting system needs to be overdamped, i. e., K_d >= 2*sqrt(K_p)
     *  \author  Robert Krug */
    class TDynLinearImpedance : public TDynPD {
    public:
    TDynLinearImpedance(std::shared_ptr<GeometricPrimitiveMap> geom_prim_map,
			std::shared_ptr<Visualizer> visualizer)
      : TDynPD(geom_prim_map, visualizer) {}

      ~TDynLinearImpedance() noexcept {}

      /*! \brief It is assumed that the two gain matrices Kp & Kd in R^{m x m} are given subsequently in row-major format in the parameter string, i. e., Kp_ = parameters[1] - parameters[m^2], and Kd_ = parameters[m^2+1] - parameters[2*m^2], where m is the task space dimension
       */
      int init(const std::vector<std::string>& parameters, RobotStatePtr robot_state, const Eigen::VectorXd& e_initial, const Eigen::VectorXd& e_dot_initial, const Eigen::VectorXd& e_final, const Eigen::VectorXd& e_dot_final);

      int update(const RobotStatePtr robot_state, const std::shared_ptr< TaskDefinition > def);
      int monitor();

    protected:
      Eigen::MatrixXd B_inv_; ///< inverse of the desired inertia matrix
	    
    private:
      TDynLinearImpedance(const TDynLinearImpedance& other) = delete;
      TDynLinearImpedance(TDynLinearImpedance&& other) = delete;
      TDynLinearImpedance& operator=(const TDynLinearImpedance& other) = delete;
      TDynLinearImpedance& operator=(TDynLinearImpedance&& other) noexcept = delete;
    };

  }  // namespace tasks

}  // namespace hiqp

#endif  // include guard
