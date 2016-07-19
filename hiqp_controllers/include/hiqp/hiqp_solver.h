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

// Eigen Includes
#include <Eigen/Dense>






namespace hiqp
{






struct HiQPStage
{
	double e_dot_star_;

	Eigen::MatrixXd J_;

	double slack;
};






class HiQPSolver
{

public:

	HiQPSolver() {}
	~HiQPSolver() noexcept {}

	virtual int solveHiQPProblem(std::vector<double>& solution) = 0;

	int setStageParameters
	(
		std::size_t priority_level, 
		double e_dot_star,
		const Eigen::MatrixXd& J
	)
	{
		StageMapIterator it = stages_map_.find(priority_level);

		if (it == stages_map_.end())
		{
			HiQPStage stage;
			stage.e_dot_star_ = e_dot_star;
			stage.J_ = J;

			stages_map_.insert(StageMapElement(priority_level, stage));
		}
		else
		{
			it->second.e_dot_star_ = e_dot_star;
			it->second.J_ = J;	
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