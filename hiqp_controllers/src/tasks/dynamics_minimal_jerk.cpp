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

/*
 * \file   dynamics_minimal_jerk.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */

#include <hiqp/hiqp_utils.h>

#include <hiqp/tasks/dynamics_minimal_jerk.h>





namespace hiqp
{
namespace tasks
{





int DynamicsMinimalJerk::init
(
  const HiQPTimePoint& sampling_time,
  const std::vector<std::string>& parameters,
  const Eigen::VectorXd& e_initial,
  const Eigen::VectorXd& e_final
)
{
  if (parameters.size() != 2)
    return -1;

  performance_measures_.resize(e_initial.rows());

  time_start_ = sampling_time;

  total_duration_ = std::stod( parameters.at(1) );

  f_ = 30 / total_duration_;

  e_diff_ = e_final - e_initial;

  std::cout << "f_ = " << f_ << "\n";
  std::cout << "e_diff_ = " << e_diff_ << "\n";

  return 0;
}





int DynamicsMinimalJerk::apply
(
  const HiQPTimePoint& sampling_time,
  const Eigen::VectorXd& e,
  const Eigen::MatrixXd& J,
  Eigen::VectorXd& e_dot_star
)
{
  double d = (sampling_time - time_start_).toSec();

  double tau = d / total_duration_;

  double t = tau*tau - 2*tau*tau*tau + tau*tau*tau*tau;

  if (tau > 1)
  {
    e_dot_star = 0*e; // first order wth gain -1
    std::cout << "e = " << e << "\n";
  }
  else
  {
    e_dot_star = f_ * e_diff_ * t; // minimal jerk
  }

  //std::cout << "total_duration_ = " << total_duration_ << "\n";
  //std::cout << "d = " << d << "\n";
  //std::cout << "tau = " << tau << "\n";

  //std::cout << "J = " << J << "\n";
  //std::cout << "I = " << Eigen::VectorXd::Ones(J.cols()) << "\n";
  //std::cout << "e_dot_star = " << e_dot_star << "\n\n\n";

  return 0;
}





int DynamicsMinimalJerk::monitor()
{
  return 0;
}





} // namespace tasks

} // namespace hiqp