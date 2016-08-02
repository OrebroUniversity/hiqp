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
 * \file   dynamics_minimal_jerk.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */


#include <hiqp/hiqp_utils.h>

#include <hiqp/tasks/dynamics_minimal_jerk.h>









namespace hiqp
{







int DynamicsMinimalJerk::init
(
    const std::vector<std::string>& parameters
)
{
    return 0;
}







int DynamicsMinimalJerk::apply
(
	const Eigen::VectorXd& e,
	Eigen::VectorXd& e_dot_star
)
{
	return 0;
}








} // namespace hiqp