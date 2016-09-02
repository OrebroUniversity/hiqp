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




/*!
 * \file   dynamics_jnt_limits.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   August, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <hiqp/hiqp_utils.h>

#include <hiqp/tasks/dynamics_jnt_limits.h>









namespace hiqp
{







int DynamicsJntLimits::init
(
	const std::chrono::steady_clock::time_point& sampling_time,
    const std::vector<std::string>& parameters,
    const Eigen::VectorXd& e_initial,
    const Eigen::VectorXd& e_final
)
{
    dq_max_ = std::stod( parameters.at(1) );
    std::cout << "dq_max_ = " << dq_max_ << "\n";

    performance_measures_.resize(0);
    
    return 0;
}







int DynamicsJntLimits::apply
(
	const std::chrono::steady_clock::time_point& sampling_time,
	const Eigen::VectorXd& e,
	const Eigen::MatrixXd& J,
	Eigen::VectorXd& e_dot_star
)
{
	e_dot_star(0) = -dq_max_;
	e_dot_star(1) = dq_max_;
	e_dot_star(2) = -e(2); // first order dynamics with gain = -1
	e_dot_star(3) = -e(3); // first order dynamics with gain = -1

	return 0;
}


int DynamicsJntLimits::monitor()
{
	return 0;
}








} // namespace hiqp