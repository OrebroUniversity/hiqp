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
 * \file   task_jnt_config_one.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/tasks/task_jnt_config_one.h>

#include <hiqp/hiqp_utils.h>


#include <iostream>






namespace hiqp
{




int TaskJntConfigOne::init
(
	const HiQPTimePoint& sampling_time,
	const std::vector<std::string>& parameters,
    const KDL::Tree& kdl_tree, 
	unsigned int num_controls
)
{

	int size = parameters.size();
	if (size != 2)
	{
		printHiqpWarning("TaskJntConfigOne requires 2 parameters, got " 
			+ std::to_string(size) + "! Initialization failed!");
		return -1;
	}

	link_name_ = parameters.at(0);

	joint_q_nr_ = kdl_getQNrFromLinkName(kdl_tree, link_name_);

	if (joint_q_nr_ < 0)
	{
		printHiqpWarning("TaskJntConfigOne::init, couldn't find joint '" + link_name_ + "'! Initialization failed.");
		return -2;
	}

	desired_configuration_ = std::stod( parameters.at(1) );

	e_.resize(1);
	J_.resize(1, num_controls);
	e_dot_star_.resize(1);
	performance_measures_.resize(0);
	task_types_.insert(task_types_.begin(), 1, 0);


	for (int i=0; i<num_controls; ++i) 
		J_(0, i) = 0;

	J_(0, joint_q_nr_) = -1;

	return 0;
}




int TaskJntConfigOne::apply
(
	const HiQPTimePoint& sampling_time,
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel
)
{
	const KDL::JntArray &q = kdl_joint_pos_vel.q;

	e_(0) = desired_configuration_ - q(joint_q_nr_);

	return 0;
}





int TaskJntConfigOne::monitor()
{
	return 0;
}






} // namespace hiqp