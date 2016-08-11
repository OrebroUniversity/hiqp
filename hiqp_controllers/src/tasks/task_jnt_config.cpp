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
 * \file   task_geometric_projection.cpp
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#include <hiqp/tasks/task_jnt_config.h>

#include <hiqp/hiqp_utils.h>


#include <iostream>






namespace hiqp
{




int TaskJntConfig::init
(
	const std::vector<std::string>& parameters,
	unsigned int num_controls
)
{
	int size = parameters.size();
	if (size != 0 && size != num_controls)
	{
		printHiqpWarning("TaskJntConfig requires 0 or " 
			+ std::to_string(num_controls) + " parameters, got " 
			+ std::to_string(size) + "! Initialization failed!");
		return -1;
	}

	if (size == 0)
	{
		desired_configuration_ = std::vector<double>(num_controls, 0);
	}
	else
	{
		desired_configuration_.resize(0);
		for (int i=0; i < num_controls; ++i)
		{
			desired_configuration_.push_back( std::stod( parameters.at(i) ) );
		}
	}


	e_.resize(1);
	J_.resize(1, num_controls);
	e_dot_star_.resize(1);
	performance_measures_.resize(1);
	task_types_.insert(task_types_.begin(), 1, 0);

	//for (int i=0; i<num_controls; ++i) 
	//	J_(0, i) = -1;

	return 0;
}




int TaskJntConfig::apply
(
	const KDL::Tree& kdl_tree, 
	const KDL::JntArrayVel& kdl_joint_pos_vel
)
{
	const KDL::JntArray &q = kdl_joint_pos_vel.q;
	e_(0) = 0;
	
	for (int i=0; i<q.rows(); ++i)
	{
		double d = desired_configuration_.at(i) - q(i); 
		e_(0) += d*d;
	}

	for (int q_nr = 0; q_nr < J_.cols(); ++q_nr)
	{
		J_(0, q_nr) = - 2 * ( desired_configuration_.at(q_nr) - q(q_nr) );
	}

	return 0;
}





int TaskJntConfig::monitor()
{
	performance_measures_.at(0) = e_(0);
	
	return 0;
}






} // namespace hiqp