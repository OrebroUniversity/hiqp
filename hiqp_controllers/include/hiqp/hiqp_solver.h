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
 * \file   hiqp_solver.h
 * \Author Marcus A Johansson (marcus.adam.johansson@gmail.com)
 * \date   July, 2016
 * \brief  Brief description of file.
 *
 * Detailed description of file.
 */



#ifndef HIQP_HIQP_SOLVER_H
#define HIQP_HIQP_SOLVER_H



// STL Includes
#include <map>
#include <vector>
#include <iostream>

// Eigen Includes
#include <Eigen/Dense>






namespace hiqp
{






struct HiQPStage
{
	int nRows;

	Eigen::VectorXd e_dot_star_;

	Eigen::MatrixXd J_;

	std::vector<int> constraint_signs_;
};




class HiQPSolver
{

public:

	HiQPSolver() {}
	~HiQPSolver() noexcept {}

	virtual int solve(std::vector<double>& solution) = 0;





	int clearStages()
	{
		stages_map_.clear();

		return 0;
	}





	int appendStage
	(
		std::size_t priority_level, 
		const Eigen::VectorXd& e_dot_star,
		const Eigen::MatrixXd& J,
		const std::vector<int>& constraint_signs
	)
	{
		StageMapIterator it = stages_map_.find(priority_level);

		// if there's no stage for this priority level
		if (it == stages_map_.end())
		{
			HiQPStage stage;
			stage.e_dot_star_ = e_dot_star;
			stage.J_ = J;
			stage.constraint_signs_ = constraint_signs;
			stage.nRows = e_dot_star.rows();

			stages_map_.insert(StageMapElement(priority_level, stage));
		}

		// else, a.k.a. there is already a stage for this priority level
		else
		{
			int rows = it->second.e_dot_star_.rows() + e_dot_star.rows();

			Eigen::VectorXd edotstar__(rows);

			edotstar__ << it->second.e_dot_star_, 
			              e_dot_star;

			it->second.e_dot_star_.resize(rows);

			it->second.e_dot_star_ = edotstar__;


			Eigen::MatrixXd J__(rows, it->second.J_.cols());
					
			J__ << it->second.J_,
			       J;

			it->second.J_ = J__;


			it->second.constraint_signs_.insert(
				it->second.constraint_signs_.begin(),
				constraint_signs.begin(),
				constraint_signs.end() );


			it->second.nRows += e_dot_star.rows();
		}

		return 0;
	}






protected:

	typedef std::map<std::size_t, HiQPStage> StageMap;
	typedef std::map<std::size_t, HiQPStage>::iterator StageMapIterator;
	typedef std::pair<std::size_t, HiQPStage> StageMapElement;

	StageMap		stages_map_; 


private:

	// No copying of this class is allowed !
    HiQPSolver(const HiQPSolver& other) = delete;
    HiQPSolver(HiQPSolver&& other) = delete;
    HiQPSolver& operator=(const HiQPSolver& other) = delete;
    HiQPSolver& operator=(HiQPSolver&& other) noexcept = delete;

};













} // namespace hiqp












#endif // include guard