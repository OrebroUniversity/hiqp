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
 * \file   task_beh_fo.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_TASK_BEHAVIOUR_FIRSTORDER_H
#define HIQP_TASK_BEHAVIOUR_FIRSTORDER_H

// HiQP Includes
#include <hiqp/task_behaviour.h>



namespace hiqp
{






/*!
 * \class TaskBehFO
 * \brief This is my awesome controller
 *
 *  It's awesome!
 */	
class TaskBehFO : public TaskBehaviour
{
public:

	TaskBehFO() {}

	~TaskBehFO() noexcept {}

	int init(const std::vector<std::string>& parameters);

	int apply
	(
		const Eigen::VectorXd& e,
		Eigen::VectorXd& e_dot_star
	);

private:

	// No copying of this class is allowed !
	TaskBehFO(const TaskBehFO& other) = delete;
	TaskBehFO(TaskBehFO&& other) = delete;
	TaskBehFO& operator=(const TaskBehFO& other) = delete;
	TaskBehFO& operator=(TaskBehFO&& other) noexcept = delete;

	double lambda_;

};







} // namespace hiqp






#endif // include guard