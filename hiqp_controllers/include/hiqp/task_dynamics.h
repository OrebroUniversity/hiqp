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
 * \file   task_dynamics.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_DYNAMICS_H
#define HIQP_TASK_DYNAMICS_H


// STL Includes
#include <vector>

// Eigen Includes
#include <Eigen/Dense>





namespace hiqp
{
	




class TaskFactory;

class TaskDynamics
{
public:

	TaskDynamics() {}
	~TaskDynamics() noexcept {}

	virtual int init(const std::vector<std::string>& parameters) = 0;

	virtual int apply
	(
		const Eigen::VectorXd& e,
		Eigen::VectorXd& e_dot_star
	) = 0;



private:

	// No copying of this class is allowed !
	TaskDynamics(const TaskDynamics& other) = delete;
	TaskDynamics(TaskDynamics&& other) = delete;
	TaskDynamics& operator=(const TaskDynamics& other) = delete;
	TaskDynamics& operator=(TaskDynamics&& other) noexcept = delete;

	friend TaskFactory;

};








} // namespace hiqp






#endif // include guard