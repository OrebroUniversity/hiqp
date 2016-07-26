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

	Eigen::MatrixXd e_dot_star_;

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
		const Eigen::MatrixXd& e_dot_star,
		const Eigen::MatrixXd& J,
		const std::vector<int>& constraint_signs
	)
	{
		StageMapIterator it = stages_map_.find(priority_level);

		if (it == stages_map_.end())
		{
			HiQPStage stage;
			stage.e_dot_star_ = e_dot_star;
			stage.J_ = J;
			stage.constraint_signs_ = constraint_signs;
			stage.nRows = e_dot_star.rows();

			stages_map_.insert(StageMapElement(priority_level, stage));
		}
		else
		{
			int rows = it->second.e_dot_star_.rows() + e_dot_star.rows();

			Eigen::MatrixXd edotstar__(rows, 1);

			edotstar__ << it->second.e_dot_star_, 
			              e_dot_star;

			it->second.e_dot_star_.resize(rows, 1);

			it->second.e_dot_star_ << edotstar__;



			Eigen::MatrixXd J__(rows, it->second.J_.cols());
			
			J__ << it->second.J_,
			       J;

			it->second.J_ << J__;



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