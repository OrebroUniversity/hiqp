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
    const std::vector<std::string>& parameters
)
{
	size_ = parameters.size()-1;

    jnt_vel_limits_.resize( size_ );

    std::transform(
    	parameters.begin()+1, 
    	parameters.end(), 
    	jnt_vel_limits_.begin(),
    	[](const std::string& param) {return std::stod(param); }
    );
    
    return 0;
}







int DynamicsJntLimits::apply
(
	const Eigen::VectorXd& e,
	Eigen::VectorXd& e_dot_star
)
{
	e_dot_star.resize( size_ );

	int i=0;
	for (double limit : jnt_vel_limits_)
	{
		e_dot_star(i) = limit;
		++i;
	}

	return 0;
}








} // namespace hiqp