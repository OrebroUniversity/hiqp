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
 * \file   dynamics_jnt_limits.h
 * \author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   August, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_DYNAMICS_JNT_LIMITS_H
#define HIQP_DYNAMICS_JNT_LIMITS_H

// HiQP Includes
#include <hiqp/hiqp_time_point.h>
#include <hiqp/task_dynamics.h>



namespace hiqp
{






/*!
 * \class DynamicsJntLimits
 * \brief 
 */	
class DynamicsJntLimits : public TaskDynamics
{
public:

	DynamicsJntLimits() {}

	~DynamicsJntLimits() noexcept {}

	int init
	(
		const HiQPTimePoint& sampling_time,
		const std::vector<std::string>& parameters,
    	const Eigen::VectorXd& e_initial,
    	const Eigen::VectorXd& e_final
	);

	int apply
	(
		const HiQPTimePoint& sampling_time,
		const Eigen::VectorXd& e,
		const Eigen::MatrixXd& J,
		Eigen::VectorXd& e_dot_star
	);

	int monitor();

private:

	// No copying of this class is allowed !
	DynamicsJntLimits(const DynamicsJntLimits& other) = delete;
	DynamicsJntLimits(DynamicsJntLimits&& other) = delete;
	DynamicsJntLimits& operator=(const DynamicsJntLimits& other) = delete;
	DynamicsJntLimits& operator=(DynamicsJntLimits&& other) noexcept = delete;

	double 						dq_max_;

};







} // namespace hiqp






#endif // include guard