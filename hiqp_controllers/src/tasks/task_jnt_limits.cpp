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
 * \file   task_jnt_limits.cpp
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/tasks/task_jnt_limits.h>

#include <hiqp/hiqp_utils.h>


#include <iostream>






namespace hiqp
{






int TaskJntLimits::init
(
	const std::vector<std::string>& parameters,
	unsigned int num_controls
)
{
	int size = parameters.size();
	if (size != 1)
	{
		printHiqpWarning("TaskJntLimits requires 1 parameter, got " 
			+ std::to_string(size) + "! Initialization failed!");
		return -1;
	}

	num_controls_ = num_controls;

	e_.resize(num_controls);
	J_.resize(num_controls, num_controls);
	e_dot_star_.resize(num_controls);
	performance_measures_.resize(num_controls);


	std::string sign = parameters.at(0);
	if (sign.compare("<") == 0 || sign.compare("<=") == 0)
	{
		task_types_.insert(task_types_.begin(), num_controls, -1);
	}
	else if (sign.compare("==") == 0 || sign.compare("=") == 0)
	{
		task_types_.insert(task_types_.begin(), num_controls, 0);
	}
	else if (sign.compare(">") == 0 || sign.compare(">=") == 0)
	{
		task_types_.insert(task_types_.begin(), num_controls, 1);
	}


	for (int i=0; i<num_controls; ++i) 
		for (int j=0; j<num_controls; ++j) 
			J_(i, j) = (i==j ? 1 : 0);

	return 0;
}




int TaskJntLimits::apply
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel
)
{
	const KDL::JntArray &q = kdl_joint_pos_vel.q;
	
	for (int i=0; i<q.rows(); ++i)
	{
		e_(i) = q(i);
	}

	return 0;
}





int TaskJntLimits::monitor()
{
	for (int i=0; i<num_controls_; ++i)
		performance_measures_.at(i) = e_(i);
	
	return 0;
}







} // namespace hiqp