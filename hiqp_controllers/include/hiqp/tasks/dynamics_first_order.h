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
 * \file   dynamics_first_order.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_DYNAMICS_FIRST_ORDER_H
#define HIQP_DYNAMICS_FIRST_ORDER_H

// HiQP Includes
#include <hiqp/task_dynamics.h>



namespace hiqp
{






/*!
 * \class DynamicsFirstOrder
 * \brief A general first-order task dynamics implementation that enforces an 
 *        exponential decay of the task function value
 */	
class DynamicsFirstOrder : public TaskDynamics
{
public:

	DynamicsFirstOrder() {}

	~DynamicsFirstOrder() noexcept {}

	int init
	(
		const std::chrono::steady_clock::time_point& sampling_time,
		const std::vector<std::string>& parameters,
    	const Eigen::VectorXd& e_initial,
    	const Eigen::VectorXd& e_final
	);

	int apply
	(
		const std::chrono::steady_clock::time_point& sampling_time,
		const Eigen::VectorXd& e,
		const Eigen::MatrixXd& J,
		Eigen::VectorXd& e_dot_star
	);

	int monitor();

private:

	// No copying of this class is allowed !
	DynamicsFirstOrder(const DynamicsFirstOrder& other) = delete;
	DynamicsFirstOrder(DynamicsFirstOrder&& other) = delete;
	DynamicsFirstOrder& operator=(const DynamicsFirstOrder& other) = delete;
	DynamicsFirstOrder& operator=(DynamicsFirstOrder&& other) noexcept = delete;

	double lambda_;

};







} // namespace hiqp






#endif // include guard