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
 * \file   dynamics_minimal_jerk.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_DYNAMICS_MINIMAL_JERK_H
#define HIQP_DYNAMICS_MINIMAL_JERK_H

// HiQP Includes
#include <hiqp/task_dynamics.h>



namespace hiqp
{






/*!
 * \class DynamicsFirstOrder
 * \brief A general first-order task dynamics implementation that enforces an 
 *        exponential decay of the task function value
 */	
class DynamicsMinimalJerk : public TaskDynamics
{
public:

	DynamicsMinimalJerk() {}

	~DynamicsMinimalJerk() noexcept {}

	int init(const std::vector<std::string>& parameters);

	int apply
	(
		const Eigen::VectorXd& e,
		Eigen::VectorXd& e_dot_star
	);

private:

	// No copying of this class is allowed !
	DynamicsMinimalJerk(const DynamicsMinimalJerk& other) = delete;
	DynamicsMinimalJerk(DynamicsMinimalJerk&& other) = delete;
	DynamicsMinimalJerk& operator=(const DynamicsMinimalJerk& other) = delete;
	DynamicsMinimalJerk& operator=(DynamicsMinimalJerk&& other) noexcept = delete;


};







} // namespace hiqp






#endif // include guard