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

#include <hiqp/tasks/tdyn_linear_impedance.h>
#include <hiqp/utilities.h>

namespace hiqp {
namespace tasks {

int TDynLinearImpedance::init(const std::vector<std::string> &parameters,
                              RobotStatePtr robot_state,
                              const Eigen::VectorXd &e_initial,
                              const Eigen::VectorXd &e_dot_initial,
                              const Eigen::VectorXd &e_final,
                              const Eigen::VectorXd &e_dot_final) {
  int dim = e_initial.rows();
  int size = parameters.size();
  assert((e_dot_initial.rows() == dim) && (e_final.rows() == dim) &&
         (e_dot_final.rows() == dim));

  if ((size != 3 * dim * dim + 1) && (size != 4)) {
    printHiqpWarning("TDynLinearImpedance for a " + std::to_string(dim) +
                     "-dimensional task requires either " +
                     std::to_string(3 * dim * dim + 1) +
                     " or 4 parameters, got " + std::to_string(size) +
                     "! Initialization failed!");
    return -1;
  }
  Eigen::MatrixXd B;
  std::vector<std::string> tdyn_pd_parameters;
  if (size == 4) {
    B = Eigen::MatrixXd::Identity(dim, dim) * std::stod(parameters.at(3));

    // extract the parameters for the base class
    for (unsigned int i = 0; i < size - 1; i++)
      tdyn_pd_parameters.push_back(parameters.at(i));
  } else { // read the full B matrix
    double *B_data = new double[dim * dim];
    for (int i = 0; i < dim * dim; i++)
      B_data[i] = std::stod(parameters.at(i + 1 + 2 * dim * dim));

    B = (Eigen::Map<Eigen::MatrixXd>(B_data, dim, dim)).transpose();
    delete B_data;

    // extract the parameters for the base class
    for (unsigned int i = 0; i < 2 * dim * dim + 1; i++)
      tdyn_pd_parameters.push_back(parameters.at(i));
  }

  // ensure that the desired inertia matrix is positive definite (should also
  // check for symmetry actually)
  Eigen::LLT<Eigen::MatrixXd> lltOfB(
      B); // compute the Cholesky decomposition of B
  if (lltOfB.info() == Eigen::NumericalIssue) {
    printHiqpWarning("The given desired inertia matrix B is not positive "
                     "definite! Initialization of TDynLinearImpedance object "
                     "failed!");
    return -1;
  }

  B_inv_ = pinv(B);

  // initialize the base class
  if (TDynPD::init(tdyn_pd_parameters, robot_state, e_initial, e_dot_initial,
                   e_final, e_dot_final) != 0)
    return -1;

  //=============Debug======================
  // std::cerr<<"e_initial: "<<e_initial.transpose()<<std::endl;
  // std::cerr<<"e_dot_initial: "<<e_dot_initial.transpose()<<std::endl;
  // std::cerr<<"e_final: "<<e_final.transpose()<<std::endl;
  // std::cerr<<"e_dot_final: "<<e_dot_final.transpose()<<std::endl;
  // std::cerr<<"dim: "<<dim<<std::endl;
  // std::cerr<<"size: "<<size<<std::endl;
  // std::cerr<<"Kp: "<<std::endl<<Kp_<<std::endl;
  // std::cerr<<"Kd: "<<std::endl<<Kd_<<std::endl;
  // std::cerr<<"B_inv: "<<std::endl<<B_inv_<<std::endl<<std::endl;
  //===========End Debug====================

  return 0;
}

int TDynLinearImpedance::update(const RobotStatePtr robot_state,
                                const TaskDefinitionPtr def) {
  // linear impedance control law
  e_ddot_star_ =
      B_inv_ * (-Kp_ * def->getTaskValue() - Kd_ * def->getTaskDerivative() +
                def->getExogeneousTaskQuantities());

  // DEBUG===================================
  // std::cerr << "e: " << def->getTaskValue().transpose() << std::endl;
  // std::cerr << "e_dot: " << def->getTaskDerivative().transpose() << std::endl;
  // std::cerr << "f: " << def->getExogeneousTaskQuantities().transpose()
  //           << std::endl;
  // std::cerr << "e_ddot_star_: " << e_ddot_star_.transpose() << std::endl;
  // std::cerr << ".........................................." << std::endl;
  // DEBUG END===============================

  return 0;
}

int TDynLinearImpedance::monitor() { return 0; }

} // namespace tasks

} // namespace hiqp
